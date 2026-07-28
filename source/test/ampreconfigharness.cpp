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

/* An activated SPS must keep the same content for the whole coded video
 * sequence.
 *
 * H.265 clause 7.4.2.4.2 states that an activated SPS RBSP remains active for
 * the entire CVS, and that any SPS NAL unit carrying the sps_seq_parameter_set_id
 * of the active SPS shall have the same content as that active SPS, unless it
 * follows the last access unit of the CVS and precedes the first VCL NAL unit
 * of another CVS. A CVS starts at an IRAP access unit with NoRaslOutputFlag
 * equal to 1 (clause 3.31 in V11, 3.28 in V1); by clause 8.1.3 (8.1 in V1) a
 * CRA picture gets NoRaslOutputFlag equal to 1 only when it is the first
 * picture in the bitstream, follows an end of sequence NAL unit, or has
 * HandleCraAsBlaFlag set by external means. With --open-gop, every keyframe
 * after the first is such a CRA, so it does not begin a new CVS.
 *
 * amp_enabled_flag is one of the SPS fields covered by that rule. The encoder's
 * part_mode path also reads the live SPS, which makes keeping the analysis
 * state aligned with the active SPS a correction constraint. This test does
 * not parse CABAC slice syntax or adjudicate a separate slice-level
 * conformance claim; it checks parameter-set stability only.
 *
 * Neither AMP nor mid-encode reconfiguration is required by H.265. The SPS
 * oracle therefore asserts the invariant independently of the remediation,
 * while a separate compatibility assertion preserves x265's successful public
 * reconfiguration return. Leaving the AMP field unchanged, deferring it to a
 * real CVS boundary, and creating such a boundary before signalling it all
 * satisfy the oracle. Returning an API error would preserve the bitstream
 * invariant but intentionally fail the compatibility assertion.
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

enum
{
    NAL_IDR_W_RADL = 19,
    NAL_IDR_N_LP   = 20,
    NAL_CRA        = 21,
    NAL_SPS        = 33,
    NAL_EOS        = 36,
    NAL_LAST_VCL   = 31,
    NAL_FIRST_IRAP = 16,
    NAL_LAST_IRAP  = 23
};

/* ------------------------------------------------------------------ reader */

class BitReader
{
public:
    BitReader(const uint8_t* data, size_t size)
        : m_data(data)
        , m_size(size)
        , m_position(0)
        , m_error(false)
    {}

    uint32_t u(uint32_t width)
    {
        uint32_t value = 0;
        if (width > 32 || m_position + width > m_size * 8)
        {
            m_error = true;
            return 0;
        }
        for (uint32_t i = 0; i < width; i++)
        {
            value <<= 1;
            value |= (m_data[m_position >> 3] >> (7 - (m_position & 7))) & 1;
            m_position++;
        }
        return value;
    }

    uint32_t ue()
    {
        uint32_t leading = 0;
        while (!m_error && !u(1))
        {
            if (++leading > 32)
            {
                m_error = true;
                return 0;
            }
        }
        if (m_error)
            return 0;
        if (!leading)
            return 0;
        return (1u << leading) - 1 + u(leading);
    }

    bool error() const { return m_error; }

private:
    const uint8_t* m_data;
    size_t         m_size;
    size_t         m_position;
    bool           m_error;
};

/* Remove nal_unit_header() and emulation_prevention_three_byte so comparisons
 * cover the SPS RBSP content named by clause 7.4.2.4.2, and nothing else. */
static void extractRbsp(const uint8_t* nal, size_t size, std::vector<uint8_t>& rbsp)
{
    rbsp.clear();
    if (size < 2)
        return;

    uint32_t zeroCount = 0;
    for (size_t i = 2; i < size; i++)
    {
        uint8_t value = nal[i];
        if (zeroCount >= 2 && value == 3)
        {
            zeroCount = 0;
            continue;
        }
        rbsp.push_back(value);
        zeroCount = value ? 0 : zeroCount + 1;
    }
}

/* clause 7.3.3 profile_tier_level( ), skipped field by field */
static void skipProfileTierLevel(BitReader& bits, uint32_t maxSubLayersMinus1)
{
    bits.u(2 + 1 + 5);                     /* space, tier, profile idc */
    bits.u(32);                            /* compatibility flags */
    bits.u(4);                             /* source and packing constraints */
    bits.u(32);                            /* reserved constraint flags */
    bits.u(11);
    bits.u(1);                             /* inbld or reserved */
    bits.u(8);                             /* general_level_idc */

    std::vector<uint32_t> profilePresent(maxSubLayersMinus1);
    std::vector<uint32_t> levelPresent(maxSubLayersMinus1);
    for (uint32_t i = 0; i < maxSubLayersMinus1; i++)
    {
        profilePresent[i] = bits.u(1);
        levelPresent[i] = bits.u(1);
    }
    if (maxSubLayersMinus1)
        for (uint32_t i = maxSubLayersMinus1; i < 8; i++)
            bits.u(2);
    for (uint32_t i = 0; i < maxSubLayersMinus1; i++)
    {
        if (profilePresent[i])
        {
            bits.u(2 + 1 + 5);
            bits.u(32);
            bits.u(4);
            bits.u(32);
            bits.u(11);
            bits.u(1);
        }
        if (levelPresent[i])
            bits.u(8);
    }
}

/* clause 7.3.2.2.1, read only as far as sps_seq_parameter_set_id */
static bool readSpsId(const std::vector<uint8_t>& rbsp, uint32_t& spsId)
{
    if (rbsp.empty())
        return false;
    BitReader bits(&rbsp[0], rbsp.size());
    bits.u(4);                                   /* sps_video_parameter_set_id */
    uint32_t maxSubLayersMinus1 = bits.u(3);
    bits.u(1);                                   /* sps_temporal_id_nesting_flag */
    skipProfileTierLevel(bits, maxSubLayersMinus1);
    spsId = bits.ue();
    return !bits.error();
}

/* ------------------------------------------------------------- NAL scanning */

struct Nal
{
    uint32_t type;
    size_t   start;      /* first byte of nal_unit_header() */
    size_t   size;
    int      cvs;        /* CVS the NAL unit belongs to, 1 based */
};

static void scanAnnexB(const std::vector<uint8_t>& stream, std::vector<Nal>& nals)
{
    nals.clear();
    std::vector<size_t> starts;
    for (size_t i = 0; i + 2 < stream.size(); i++)
    {
        if (!stream[i] && !stream[i + 1] && stream[i + 2] == 1)
        {
            starts.push_back(i + 3);
            i += 2;
        }
    }
    for (size_t i = 0; i < starts.size(); i++)
    {
        size_t begin = starts[i];
        size_t end = (i + 1 < starts.size()) ? starts[i + 1] - 3 : stream.size();
        while (end > begin && !stream[end - 1] && i + 1 < starts.size())
            end--;                                  /* zero byte of the next start code */
        if (end < begin + 2)
            continue;
        Nal nal;
        nal.type = (stream[begin] >> 1) & 0x3F;
        nal.start = begin;
        nal.size = end - begin;
        nal.cvs = 0;
        nals.push_back(nal);
    }
}

/* Clauses 3.31 and 8.1.3: assign every NAL unit to a CVS. A parameter set is
 * attributed to the CVS of the next VCL NAL unit in decoding order, which is
 * exactly the distinction clause 7.4.2.4.2 draws when it allows a changed
 * same-identifier SPS between two coded video sequences. */
static int deriveCvs(std::vector<Nal>& nals, int& idrCount, int& craCount, int& eosCount)
{
    idrCount = craCount = eosCount = 0;

    bool firstPictureSeen = false;
    bool eosPending = false;
    int cvs = 0;
    for (size_t i = 0; i < nals.size(); i++)
    {
        uint32_t type = nals[i].type;
        if (type == NAL_EOS)
        {
            eosCount++;
            eosPending = true;
            continue;
        }
        if (type > NAL_LAST_VCL)
            continue;

        bool isIrap = type >= NAL_FIRST_IRAP && type <= NAL_LAST_IRAP;
        bool isIdrOrBla = type >= NAL_FIRST_IRAP && type <= NAL_IDR_N_LP;
        if (type == NAL_IDR_W_RADL || type == NAL_IDR_N_LP)
            idrCount++;
        if (type == NAL_CRA)
            craCount++;
        if (isIrap && (isIdrOrBla || !firstPictureSeen || eosPending))
            cvs++;                                  /* NoRaslOutputFlag is 1 */
        firstPictureSeen = true;
        eosPending = false;
        nals[i].cvs = cvs ? cvs : 1;
    }

    /* Backward pass: a non-VCL NAL unit shares the CVS of the next VCL one. */
    int next = cvs ? cvs : 1;
    for (size_t i = nals.size(); i-- > 0;)
    {
        if (nals[i].type <= NAL_LAST_VCL)
            next = nals[i].cvs;
        else
            nals[i].cvs = next;
    }
    return cvs;
}

/* ------------------------------------------------------------------ encoder */

struct Case
{
    const char* name;
    int         openGop;
    int         reconfigAmp;
    const char* what;
};

static const int FRAME_COUNT = 6;
static const int WIDTH = 128;
static const int HEIGHT = 128;

template<typename pixel>
static void fillPicture(pixel* luma, pixel* cb, pixel* cr, int n, int maxValue)
{
    const int rx = (n * 9) % (WIDTH - 32);
    const int ry = (n * 5) % (HEIGHT - 24);
    for (int j = 0; j < HEIGHT; j++)
    {
        for (int i = 0; i < WIDTH; i++)
        {
            int value = (i * 3 + j * 5 + n * 16) & 0xFF;
            if (i >= rx && i < rx + 32 && j >= ry && j < ry + 24)
                value = 235 - (((i - rx) * 2) & 0x3F);
            luma[j * WIDTH + i] = (pixel)(value * maxValue / 255);
        }
    }
    for (int j = 0; j < HEIGHT / 2; j++)
    {
        for (int i = 0; i < WIDTH / 2; i++)
        {
            cb[j * (WIDTH / 2) + i] = (pixel)((120 + ((i + n) & 0x0F)) * maxValue / 255);
            cr[j * (WIDTH / 2) + i] = (pixel)((136 - ((j + n) & 0x0F)) * maxValue / 255);
        }
    }
}

static void appendNals(std::vector<uint8_t>& stream, x265_nal* nals, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
        stream.insert(stream.end(), nals[i].payload, nals[i].payload + nals[i].sizeBytes);
}

/* Encode FRAME_COUNT pictures, reconfiguring AMP once after the first coded
 * picture has been returned. */
static bool encodeWithReconfig(const Case& test, std::vector<uint8_t>& stream,
                               int& reconfigResult, int& codedPictures,
                               const char*& failure)
{
    stream.clear();
    reconfigResult = -1;
    codedPictures = 0;

    x265_param* param = x265_param_alloc();
    if (!param)
    {
        failure = "x265_param_alloc failed";
        return false;
    }
    x265_param_default(param);
    param->sourceWidth = WIDTH;
    param->sourceHeight = HEIGHT;
    param->internalCsp = X265_CSP_I420;
    param->internalBitDepth = X265_DEPTH;
    param->fpsNum = 25;
    param->fpsDenom = 1;
    param->bRepeatHeaders = 1;
    param->keyframeMax = 2;
    param->keyframeMin = 2;
    param->bframes = 0;
    param->bOpenGOP = test.openGop;
    param->bEnableRectInter = 1;
    param->bEnableAMP = 0;
    param->frameNumThreads = 1;
    param->lookaheadDepth = 0;      /* return coded pictures during the feed */
    param->rc.rateControlMode = X265_RC_CQP;
    param->rc.qp = 30;
    param->logLevel = X265_LOG_ERROR;

    x265_encoder* encoder = x265_encoder_open(param);
    if (!encoder)
    {
        x265_param_free(param);
        failure = "configuration declined by x265_encoder_open";
        return false;
    }

    const size_t pixelBytes = (X265_DEPTH > 8) ? 2 : 1;
    const size_t lumaSamples = (size_t)WIDTH * HEIGHT;
    std::vector<uint8_t> luma(lumaSamples * pixelBytes);
    std::vector<uint8_t> cb(lumaSamples * pixelBytes / 4);
    std::vector<uint8_t> cr(lumaSamples * pixelBytes / 4);

    bool ok = true;
    bool reconfigured = false;
    x265_nal* nals = NULL;
    uint32_t numNal = 0;

    for (int f = 0; f < FRAME_COUNT && ok; f++)
    {
        if (X265_DEPTH > 8)
            fillPicture<uint16_t>((uint16_t*)&luma[0], (uint16_t*)&cb[0], (uint16_t*)&cr[0],
                                  f, (1 << X265_DEPTH) - 1);
        else
            fillPicture<uint8_t>(&luma[0], &cb[0], &cr[0], f, 255);

        x265_picture picture;
        x265_picture_init(param, &picture);
        picture.planes[0] = &luma[0];
        picture.planes[1] = &cb[0];
        picture.planes[2] = &cr[0];
        picture.stride[0] = (int)(WIDTH * pixelBytes);
        picture.stride[1] = (int)(WIDTH * pixelBytes / 2);
        picture.stride[2] = picture.stride[1];
        picture.pts = f;

        nals = NULL;
        numNal = 0;
        int ret = x265_encoder_encode(encoder, &nals, &numNal, &picture, NULL);
        if (ret < 0)
        {
            failure = "x265_encoder_encode failed";
            ok = false;
            break;
        }
        if (ret > 0)
        {
            appendNals(stream, nals, numNal);
            codedPictures += ret;
        }

        if (ret > 0 && !reconfigured)
        {
            x265_param next;
            x265_encoder_parameters(encoder, &next);
            next.bEnableAMP = test.reconfigAmp;
            reconfigResult = x265_encoder_reconfig(encoder, &next);
            reconfigured = true;
        }
    }

    while (ok)
    {
        nals = NULL;
        numNal = 0;
        int ret = x265_encoder_encode(encoder, &nals, &numNal, NULL, NULL);
        if (ret < 0)
        {
            failure = "x265_encoder_encode failed while flushing";
            ok = false;
            break;
        }
        appendNals(stream, nals, numNal);
        codedPictures += ret;
        if (!ret)
            break;
    }

    if (ok && !reconfigured)
    {
        failure = "no coded picture was returned, the reconfiguration never happened";
        ok = false;
    }

    x265_encoder_close(encoder);
    x265_param_free(param);
    return ok;
}

} // namespace

int main()
{
    static const Case cases[] =
    {
        /* The trigger: with an open GOP the keyframes after the first are CRA
         * pictures that stay inside the CVS opened by the initial IDR. */
        { "trigger-opengop-amp-on", 1, 1,
          "open GOP, AMP 0 -> 1 while SPS id 0 is active" },
        /* Neighbouring valid behaviour: the same accepted call with nothing to
         * change must keep emitting one stable SPS. */
        { "control-opengop-amp-same", 1, 0,
          "open GOP, AMP 0 -> 0" },
        /* Boundary case: every keyframe is an IDR, so each one starts a new
         * CVS and a changed SPS in front of it is allowed by clause 7.4.2.4.2.
         * The invariant still has to hold inside each CVS. */
        { "control-closedgop-amp-on", 0, 1,
          "closed GOP, AMP 0 -> 1" },
        { "control-closedgop-amp-same", 0, 0,
          "closed GOP, AMP 0 -> 0" },
    };

    int failures = 0;
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++)
    {
        const Case& test = cases[c];
        std::vector<uint8_t> stream;
        std::vector<Nal> nals;
        const char* failure = NULL;
        int reconfigResult = -1;
        int codedPictures = 0;
        bool passed = true;
        char detail[512];
        detail[0] = 0;

        if (!encodeWithReconfig(test, stream, reconfigResult, codedPictures, failure))
        {
            /* The checked x265 configuration is a supported characterization
             * fixture. Treat setup or encode failure as a failed test rather
             * than allowing a missing stream to masquerade as conformance. */
            passed = false;
            snprintf(detail, sizeof(detail), "%s", failure);
        }
        else
        {
            scanAnnexB(stream, nals);
            int idrCount = 0, craCount = 0, eosCount = 0;
            int cvsCount = deriveCvs(nals, idrCount, craCount, eosCount);

            if (reconfigResult != 0)
            {
                passed = false;
                snprintf(detail, sizeof(detail),
                         "x265_encoder_reconfig returned %d, expected 0", reconfigResult);
            }
            else if (codedPictures != FRAME_COUNT)
            {
                passed = false;
                snprintf(detail, sizeof(detail),
                         "%d coded picture(s) returned, expected %d",
                         codedPictures, FRAME_COUNT);
            }
            else if (!cvsCount)
            {
                passed = false;
                snprintf(detail, sizeof(detail), "no CVS was started");
            }
            else
            {
                /* The invariant: inside one CVS, every SPS NAL unit carrying a
                 * given sps_seq_parameter_set_id must have the same content. */
                size_t offenders = 0;
                uint32_t changedId = 0;
                int changedCvs = 0;
                for (size_t i = 0; i < nals.size() && !offenders; i++)
                {
                    if (nals[i].type != NAL_SPS)
                        continue;
                    std::vector<uint8_t> first;
                    extractRbsp(&stream[nals[i].start], nals[i].size, first);
                    uint32_t firstId = 0;
                    if (!readSpsId(first, firstId))
                    {
                        passed = false;
                        snprintf(detail, sizeof(detail), "SPS at NAL %d could not be parsed",
                                 (int)i);
                        break;
                    }
                    for (size_t j = i + 1; j < nals.size(); j++)
                    {
                        if (nals[j].type != NAL_SPS || nals[j].cvs != nals[i].cvs)
                            continue;
                        std::vector<uint8_t> other;
                        extractRbsp(&stream[nals[j].start], nals[j].size, other);
                        uint32_t otherId = 0;
                        if (!readSpsId(other, otherId) || otherId != firstId)
                            continue;
                        if (first != other)
                        {
                            offenders++;
                            changedId = firstId;
                            changedCvs = nals[i].cvs;
                            break;
                        }
                    }
                }

                if (offenders)
                {
                    passed = false;
                    snprintf(detail, sizeof(detail),
                             "sps_seq_parameter_set_id %u changes content inside CVS %d "
                             "(%d IDR, %d CRA, %d EOS, %d CVS)",
                             changedId, changedCvs, idrCount, craCount, eosCount, cvsCount);
                }
                else if (passed)
                    snprintf(detail, sizeof(detail),
                             "reconfig=0, %d picture(s), %d IDR, %d CRA, %d EOS, %d CVS, "
                             "stable SPS content per CVS",
                             codedPictures, idrCount, craCount, eosCount, cvsCount);
            }
        }

        if (!passed)
            failures++;
        printf("active SPS stability, %s (%s, %d-bit): %s [%s]\n",
               test.name, test.what, X265_DEPTH, passed ? "PASS" : "FAIL", detail);
    }

    printf("active parameter set stability (%d-bit): %s [%d case(s) failed]\n",
           X265_DEPTH, failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
