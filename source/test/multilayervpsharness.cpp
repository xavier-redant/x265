/*****************************************************************************
 * Copyright (C) 2026 Xavier Redant
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at license @ x265.com.
 *****************************************************************************/

/* An emitted video parameter set must be a conforming video parameter set.
 *
 * H.265 clause 7.3.2.1 writes exactly one layer_id_included_flag[ i ][ j ] for
 * each j from 0 through vps_max_layer_id, for each additional layer set, and
 * places vps_timing_info_present_flag at the bit immediately after the last of
 * them. Clause 7.4.3.1 then bounds vps_num_hrd_parameters by
 * vps_num_layer_sets_minus1 + 1 and NumLayersInIdList[ i ] by
 * vps_max_layers_minus1 + 1.
 *
 * Neither alpha coding nor multiview coding is required of an encoder, and
 * neither is their combination. An encoder is free to decline any of these
 * configurations: declining emits no parameter set and cannot be
 * non-conforming. What it is not free to do is accept a configuration and then
 * emit a video parameter set that the syntax cannot decode.
 *
 * This harness therefore asserts the normative invariant rather than any
 * particular remediation: for every configuration below, if
 * x265_encoder_open() succeeds then every video parameter set returned by
 * x265_encoder_headers() must decode through rbsp_trailing_bits() with every
 * applicable bound satisfied.
 *
 * Declared limit: a bit-position desync is not guaranteed to be detectable
 * from the decoded field values alone, because with vps_extension_flag equal
 * to 1 the while( more_rbsp_data( ) ) vps_extension_data_flag loop of clause
 * 7.3.2.1 absorbs any surplus trailing bits. The invariant asserted here is
 * conformance of the emitted parameter set, which is the requirement that
 * actually binds, not an internal count that the bitstream does not carry.
 *
 * The bitstream is analysed by a reader written for this test. It shares no
 * code with the encoder that produced the stream. */

#include "common.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>

using namespace X265_NS;

namespace {

/* ------------------------------------------------------------------ reader */

class BitReader
{
public:
    BitReader(const uint8_t* data, size_t size)
        : m_data(data)
        , m_bits(size * 8)
        , m_pos(0)
        , m_overrun(false)
    {
    }

    bool overrun() const     { return m_overrun; }
    size_t position() const  { return m_pos; }
    size_t bits() const      { return m_bits; }

    uint32_t bitAt(size_t index) const
    {
        return (m_data[index >> 3] >> (7 - (index & 7))) & 1;
    }

    uint32_t read(uint32_t count)
    {
        if (m_pos + count > m_bits)
        {
            m_overrun = true;
            m_pos = m_bits;
            return 0;
        }
        uint32_t value = 0;
        for (uint32_t i = 0; i < count; i++)
            value = (value << 1) | bitAt(m_pos++);
        return value;
    }

    /* ue(v), clause 9.2 */
    uint32_t readUvlc()
    {
        uint32_t leadingZeroBits = 0;
        while (!m_overrun)
        {
            if (m_pos >= m_bits)
            {
                m_overrun = true;
                return 0;
            }
            if (read(1))
                break;
            if (++leadingZeroBits > 32)
            {
                m_overrun = true;
                return 0;
            }
        }
        if (m_overrun || !leadingZeroBits)
            return m_overrun ? 0 : 0;
        return ((uint32_t)1 << leadingZeroBits) - 1 + read(leadingZeroBits);
    }

    bool byteAligned() const { return !(m_pos % 8); }

    /* clause 7.2: is there data before the rbsp_stop_one_bit? */
    bool moreRbspData() const
    {
        if (m_pos >= m_bits)
            return false;
        for (size_t i = m_bits; i-- > 0; )
            if (bitAt(i))
                return m_pos < i;
        return false;
    }

private:
    const uint8_t* m_data;
    size_t m_bits;
    size_t m_pos;
    bool m_overrun;
};

/* Remove emulation_prevention_three_byte, clause 7.3.1.1 */
void extractRbsp(const uint8_t* payload, size_t size, std::vector<uint8_t>& rbsp)
{
    int zeros = 0;
    for (size_t i = 0; i < size; i++)
    {
        uint8_t byte = payload[i];
        if (zeros >= 2 && byte == 0x03 && (i + 1 >= size || payload[i + 1] <= 0x03))
        {
            zeros = 0;
            continue;
        }
        rbsp.push_back(byte);
        zeros = (byte == 0x00) ? zeros + 1 : 0;
    }
}

/* clause 7.3.3 */
void parseProfileTierLevel(BitReader& bs, uint32_t maxNumSubLayersMinus1)
{
    bs.read(2);              /* general_profile_space */
    bs.read(1);              /* general_tier_flag */
    bs.read(5);              /* general_profile_idc */
    for (int i = 0; i < 32; i++)
        bs.read(1);          /* general_profile_compatibility_flag */
    bs.read(4);              /* progressive / interlaced / non_packed / frame_only */
    /* Every branch of the constraint-flag conditional occupies 43 bits, then
     * general_inbld_flag or general_reserved_zero_bit. */
    bs.read(22);
    bs.read(21);
    bs.read(1);
    bs.read(8);              /* general_level_idc */

    std::vector<uint32_t> subProfile(maxNumSubLayersMinus1);
    std::vector<uint32_t> subLevel(maxNumSubLayersMinus1);
    for (uint32_t i = 0; i < maxNumSubLayersMinus1; i++)
    {
        subProfile[i] = bs.read(1);
        subLevel[i] = bs.read(1);
    }
    if (maxNumSubLayersMinus1)
        for (uint32_t i = maxNumSubLayersMinus1; i < 8; i++)
            bs.read(2);      /* reserved_zero_2bits */
    for (uint32_t i = 0; i < maxNumSubLayersMinus1; i++)
    {
        if (subProfile[i])
        {
            bs.read(2); bs.read(1); bs.read(5);
            for (int j = 0; j < 32; j++)
                bs.read(1);
            bs.read(4);
            bs.read(22); bs.read(21); bs.read(1);
        }
        if (subLevel[i])
            bs.read(8);      /* sub_layer_level_idc */
    }
}

/* clause E.2.3 */
void parseSubLayerHrdParameters(BitReader& bs, uint32_t cpbCnt, uint32_t subPicPresent)
{
    for (uint32_t i = 0; i < cpbCnt; i++)
    {
        bs.readUvlc();       /* bit_rate_value_minus1 */
        bs.readUvlc();       /* cpb_size_value_minus1 */
        if (subPicPresent)
        {
            bs.readUvlc();   /* cpb_size_du_value_minus1 */
            bs.readUvlc();   /* bit_rate_du_value_minus1 */
        }
        bs.read(1);          /* cbr_flag */
    }
}

/* clause E.2.2 */
bool parseHrdParameters(BitReader& bs, uint32_t commonInfPresent, uint32_t maxNumSubLayersMinus1,
                        char* failure, size_t failureSize)
{
    uint32_t nalHrd = 0, vclHrd = 0, subPic = 0;
    if (commonInfPresent)
    {
        nalHrd = bs.read(1);
        vclHrd = bs.read(1);
        if (nalHrd || vclHrd)
        {
            subPic = bs.read(1);
            if (subPic)
            {
                bs.read(8); bs.read(5); bs.read(1); bs.read(5);
            }
            bs.read(4); bs.read(4);
            if (subPic)
                bs.read(4);
            bs.read(5); bs.read(5); bs.read(5);
        }
    }
    for (uint32_t i = 0; i <= maxNumSubLayersMinus1; i++)
    {
        uint32_t fixedGeneral = bs.read(1);
        uint32_t fixedWithinCvs = fixedGeneral ? 1 : bs.read(1);
        uint32_t lowDelay = 0;
        if (fixedWithinCvs)
            bs.readUvlc();   /* elemental_duration_in_tc_minus1 */
        else
            lowDelay = bs.read(1);
        uint32_t cpbCntMinus1 = 0;
        if (!lowDelay)
        {
            cpbCntMinus1 = bs.readUvlc();
            if (cpbCntMinus1 > 31)
            {
                snprintf(failure, failureSize,
                         "cpb_cnt_minus1 %u is above the range 0..31 of clause E.3.2",
                         cpbCntMinus1);
                return false;
            }
        }
        if (nalHrd)
            parseSubLayerHrdParameters(bs, cpbCntMinus1 + 1, subPic);
        if (vclHrd)
            parseSubLayerHrdParameters(bs, cpbCntMinus1 + 1, subPic);
        if (bs.overrun())
        {
            snprintf(failure, failureSize, "hrd_parameters( ) ran past the end of the RBSP");
            return false;
        }
    }
    return true;
}

struct VpsSummary
{
    uint32_t maxLayersMinus1;
    uint32_t maxLayerId;
    uint32_t numLayerSetsMinus1;
    uint32_t timingInfoPresent;
    uint32_t extensionFlag;
    size_t   timingFlagBitOffset;
};

/* clause 7.3.2.1 */
bool parseVps(const std::vector<uint8_t>& rbsp, VpsSummary& out, char* failure, size_t failureSize)
{
    if (rbsp.size() < 3)
    {
        snprintf(failure, failureSize, "VPS RBSP is only %d byte(s)", (int)rbsp.size());
        return false;
    }
    /* skip the two-byte NAL unit header */
    BitReader bs(&rbsp[2], rbsp.size() - 2);

    bs.read(4);                                        /* vps_video_parameter_set_id */
    uint32_t baseLayerInternal = bs.read(1);           /* vps_base_layer_internal_flag */
    bs.read(1);                                        /* vps_base_layer_available_flag */
    out.maxLayersMinus1 = bs.read(6);                  /* vps_max_layers_minus1 */
    uint32_t maxSubLayersMinus1 = bs.read(3);          /* vps_max_sub_layers_minus1 */
    bs.read(1);                                        /* vps_temporal_id_nesting_flag */
    if (bs.read(16) != 0xFFFF)
    {
        snprintf(failure, failureSize, "vps_reserved_0xffff_16bits is not 0xffff");
        return false;
    }

    parseProfileTierLevel(bs, maxSubLayersMinus1);

    uint32_t orderingInfoPresent = bs.read(1);
    uint32_t first = orderingInfoPresent ? 0 : maxSubLayersMinus1;
    for (uint32_t i = first; i <= maxSubLayersMinus1; i++)
    {
        bs.readUvlc();       /* vps_max_dec_pic_buffering_minus1[ i ] */
        bs.readUvlc();       /* vps_max_num_reorder_pics[ i ] */
        bs.readUvlc();       /* vps_max_latency_increase_plus1[ i ] */
    }

    out.maxLayerId = bs.read(6);                       /* vps_max_layer_id */
    out.numLayerSetsMinus1 = bs.readUvlc();            /* vps_num_layer_sets_minus1 */
    if (bs.overrun())
    {
        snprintf(failure, failureSize, "the RBSP ended before vps_num_layer_sets_minus1");
        return false;
    }

    /* Exactly vps_max_layer_id + 1 flags per additional layer set, then the
     * timing flag at the very next bit. */
    for (uint32_t i = 1; i <= out.numLayerSetsMinus1; i++)
    {
        uint32_t numLayersInIdList = 0;
        for (uint32_t j = 0; j <= out.maxLayerId; j++)
            numLayersInIdList += bs.read(1);           /* layer_id_included_flag[ i ][ j ] */
        if (bs.overrun())
        {
            snprintf(failure, failureSize,
                     "the RBSP ended inside layer_id_included_flag[ %u ]", i);
            return false;
        }
        if (numLayersInIdList < 1 || numLayersInIdList > out.maxLayersMinus1 + 1)
        {
            snprintf(failure, failureSize,
                     "NumLayersInIdList[ %u ] is %u, outside the range 1..%u of clause 7.4.3.1",
                     i, numLayersInIdList, out.maxLayersMinus1 + 1);
            return false;
        }
    }

    out.timingFlagBitOffset = bs.position();
    out.timingInfoPresent = bs.read(1);                /* vps_timing_info_present_flag */
    if (out.timingInfoPresent)
    {
        bs.read(32);                                   /* vps_num_units_in_tick */
        bs.read(32);                                   /* vps_time_scale */
        if (bs.read(1))                                /* vps_poc_proportional_to_timing_flag */
            bs.readUvlc();                             /* vps_num_ticks_poc_diff_one_minus1 */
        uint32_t numHrd = bs.readUvlc();               /* vps_num_hrd_parameters */
        if (bs.overrun())
        {
            snprintf(failure, failureSize, "the RBSP ended inside the timing information");
            return false;
        }
        if (numHrd > out.numLayerSetsMinus1 + 1)
        {
            snprintf(failure, failureSize,
                     "vps_num_hrd_parameters %u is above the maximum "
                     "vps_num_layer_sets_minus1 + 1 = %u of clause 7.4.3.1",
                     numHrd, out.numLayerSetsMinus1 + 1);
            return false;
        }
        for (uint32_t i = 0; i < numHrd; i++)
        {
            uint32_t layerSetIdx = bs.readUvlc();      /* hrd_layer_set_idx[ i ] */
            uint32_t low = baseLayerInternal ? 0 : 1;
            if (layerSetIdx < low || layerSetIdx > out.numLayerSetsMinus1)
            {
                snprintf(failure, failureSize,
                         "hrd_layer_set_idx[ %u ] is %u, outside the range %u..%u of clause 7.4.3.1",
                         i, layerSetIdx, low, out.numLayerSetsMinus1);
                return false;
            }
            uint32_t cprmsPresent = i ? bs.read(1) : 1;
            if (!parseHrdParameters(bs, cprmsPresent, maxSubLayersMinus1, failure, failureSize))
                return false;
        }
    }

    out.extensionFlag = bs.read(1);                    /* vps_extension_flag */
    if (out.extensionFlag)
        while (bs.moreRbspData())
            bs.read(1);                                /* vps_extension_data_flag */

    /* rbsp_trailing_bits( ), clause 7.3.2.11 */
    if (bs.overrun())
    {
        snprintf(failure, failureSize, "the RBSP ended before rbsp_trailing_bits( )");
        return false;
    }
    if (bs.read(1) != 1)
    {
        snprintf(failure, failureSize, "rbsp_stop_one_bit is not 1");
        return false;
    }
    while (!bs.byteAligned())
    {
        if (bs.read(1))
        {
            snprintf(failure, failureSize, "rbsp_alignment_zero_bit is not 0");
            return false;
        }
    }
    if (bs.overrun() || bs.position() != bs.bits())
    {
        snprintf(failure, failureSize,
                 "%d bit(s) remain after rbsp_trailing_bits( )",
                 (int)(bs.bits() - bs.position()));
        return false;
    }
    return true;
}

/* --------------------------------------------------------------- scenarios */

struct TestCase
{
    const char* name;
    int         alpha;
    int         views;
};

const TestCase g_cases[] =
{
    { "base",              0, 1 },
    { "alpha only",        1, 1 },
    { "multiview only",    0, 2 },
    { "alpha + multiview", 1, 2 },
};

/* A valid neighbouring configuration must remain accepted. Refusal is allowed
 * only when an enabled feature is already invalid at this bit depth, or for
 * the unsupported alpha-plus-multiview intersection under test. */
bool refusalIsAllowed(const TestCase& test)
{
    if (test.alpha && ENABLE_ALPHA && X265_DEPTH > 10)
        return true;
    if (test.views > 1 && ENABLE_MULTIVIEW && X265_DEPTH != 8)
        return true;
    return test.alpha && test.views > 1 && ENABLE_ALPHA && ENABLE_MULTIVIEW;
}

/* Collect every VPS NAL unit of an emitted header set and check each one. */
bool checkHeaders(const x265_nal* nals, uint32_t numNal, int& vpsCount,
                  VpsSummary& last, char* failure, size_t failureSize)
{
    vpsCount = 0;
    for (uint32_t i = 0; i < numNal; i++)
    {
        if (nals[i].type != NAL_UNIT_VPS)
            continue;
        vpsCount++;

        const uint8_t* payload = nals[i].payload;
        uint32_t size = nals[i].sizeBytes;
        /* step over the Annex-B start code prefix */
        uint32_t offset = 0;
        while (offset + 2 < size && !(payload[offset] == 0 && payload[offset + 1] == 0 &&
                                      payload[offset + 2] == 1))
            offset++;
        if (offset + 2 >= size)
        {
            snprintf(failure, failureSize, "VPS NAL unit has no start code prefix");
            return false;
        }
        offset += 3;

        std::vector<uint8_t> rbsp;
        extractRbsp(payload + offset, size - offset, rbsp);
        if (!parseVps(rbsp, last, failure, failureSize))
            return false;
    }
    if (!vpsCount)
    {
        snprintf(failure, failureSize, "no VPS NAL unit in the emitted header set");
        return false;
    }
    return true;
}

bool runCase(const TestCase& test, int& vpsCount, VpsSummary& summary,
             char* failure, size_t failureSize)
{
    x265_param* param = x265_param_alloc();
    if (!param)
    {
        snprintf(failure, failureSize, "x265_param_alloc failed");
        return false;
    }
    x265_param_default(param);

    param->internalCsp = X265_CSP_I420;
    param->sourceWidth = 64;
    param->sourceHeight = 64;
    param->fpsNum = 25;
    param->fpsDenom = 1;
    param->logLevel = X265_LOG_NONE;
    param->bRepeatHeaders = 0;

    if (test.alpha)
    {
        param->bEnableAlpha = 1;
        param->numScalableLayers = 2;
        param->numLayers = 2;
    }
    if (test.views > 1)
        param->numViews = test.views;

    x265_encoder* encoder = x265_encoder_open(param);
    if (!encoder)
    {
        x265_param_free(param);
        snprintf(failure, failureSize, "configuration declined by x265_encoder_open");
        return false;
    }

    x265_nal* nals = NULL;
    uint32_t numNal = 0;
    int bytes = x265_encoder_headers(encoder, &nals, &numNal);
    bool ok;
    if (bytes <= 0 || !nals || !numNal)
    {
        snprintf(failure, failureSize, "x265_encoder_headers returned %d", bytes);
        ok = false;
    }
    else
        ok = checkHeaders(nals, numNal, vpsCount, summary, failure, failureSize);

    x265_encoder_close(encoder);
    x265_param_free(param);
    return ok;
}

} // namespace

int main(int, char**)
{
    int failures = 0;

    for (size_t i = 0; i < sizeof(g_cases) / sizeof(g_cases[0]); i++)
    {
        const TestCase& test = g_cases[i];
        char failure[256] = "";
        char detail[512];
        VpsSummary summary;
        int vpsCount = 0;
        bool passed;

        memset(&summary, 0, sizeof(summary));

        if (!runCase(test, vpsCount, summary, failure, sizeof(failure)))
        {
            bool declined = !strcmp(failure, "configuration declined by x265_encoder_open");
            passed = declined && refusalIsAllowed(test);
            if (declined && !passed)
                snprintf(detail, sizeof(detail),
                         "valid neighbouring configuration was unexpectedly declined");
            else
                snprintf(detail, sizeof(detail), "%s", failure);
        }
        else
        {
            passed = true;
            snprintf(detail, sizeof(detail),
                     "%d VPS: vps_max_layers_minus1=%u vps_max_layer_id=%u "
                     "vps_num_layer_sets_minus1=%u timing=%u@bit%d extension=%u",
                     vpsCount, summary.maxLayersMinus1, summary.maxLayerId,
                     summary.numLayerSetsMinus1, summary.timingInfoPresent,
                     (int)summary.timingFlagBitOffset, summary.extensionFlag);
        }

        if (!passed)
            failures++;
        printf("conforming VPS, %s (alpha=%d views=%d, %d-bit): %s [%s]\n",
               test.name, test.alpha, test.views, X265_DEPTH,
               passed ? "PASS" : "FAIL", detail);
    }

    printf("multilayer VPS conformance (%d-bit, ENABLE_ALPHA=%d ENABLE_MULTIVIEW=%d): "
           "%s [%d case(s) failed]\n",
           X265_DEPTH, ENABLE_ALPHA, ENABLE_MULTIVIEW,
           failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
