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

/* Emitted slice segments must not be empty.
 *
 * H.265 clause 7.3.2.9 defines slice_segment_layer_rbsp() as a slice segment
 * header followed by slice_segment_data() and its trailing bits, and clause
 * 7.3.8.1 codes slice_segment_data() with a do/while loop that always emits at
 * least one coding_tree_unit() and one end_of_slice_segment_flag. Clause 6.3.1
 * likewise makes every slice segment an integer number of coding tree units.
 *
 * An encoder is free to refuse multiple slices or to normalize the request to
 * a single slice; neither multiple slices nor wavefront parallel processing is
 * required of an encoder. What is not free is emitting a slice segment that
 * declares an address and then carries no coding tree unit at all.
 *
 * This harness therefore asserts the normative invariant rather than any
 * particular remediation: for every configuration below, every slice segment
 * that actually reaches the bitstream must carry at least one byte of
 * slice_segment_data() after its byte-aligned header. A configuration that the
 * encoder declines to open emits nothing and satisfies the invariant too.
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
    BitReader(const uint8_t* data, size_t size, size_t startBit)
        : m_data(data)
        , m_size(size)
        , m_position(startBit)
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
        if (m_error || !leading)
            return 0;
        return (1u << leading) - 1 + u(leading);
    }

    int32_t se()
    {
        uint32_t k = ue();
        int32_t value = (int32_t)((k + 1) >> 1);
        return (k & 1) ? value : -value;
    }

    bool byteAligned() const { return !(m_position & 7); }
    size_t position() const  { return m_position; }
    bool error() const       { return m_error; }
    void fail()              { m_error = true; }

private:
    const uint8_t* m_data;
    size_t         m_size;
    size_t         m_position;
    bool           m_error;
};

/* Remove emulation_prevention_three_byte, keeping nal_unit_header() intact. */
static void extractRbsp(const uint8_t* nal, size_t size, std::vector<uint8_t>& rbsp)
{
    rbsp.clear();
    uint32_t zeroCount = 0;
    for (size_t i = 0; i < size; i++)
    {
        uint8_t value = nal[i];
        if (i >= 2 && zeroCount >= 2 && value == 3)
        {
            zeroCount = 0;
            continue;
        }
        rbsp.push_back(value);
        zeroCount = value ? 0 : zeroCount + 1;
    }
}

/* ------------------------------------------------------ parameter set state */

struct SpsInfo
{
    uint32_t picSizeInCtbs;
    uint32_t addressBits;
    uint32_t chromaArrayType;
    uint32_t separateColourPlaneFlag;
    uint32_t saoEnabledFlag;
    bool     valid;

    SpsInfo() { memset(this, 0, sizeof(*this)); }
};

struct PpsInfo
{
    uint32_t dependentSlicesEnabledFlag;
    uint32_t outputFlagPresentFlag;
    uint32_t numExtraSliceHeaderBits;
    uint32_t sliceChromaQpOffsetsPresentFlag;
    uint32_t tilesEnabledFlag;
    uint32_t entropyCodingSyncEnabledFlag;
    uint32_t loopFilterAcrossSlicesEnabledFlag;
    uint32_t deblockingOverrideEnabledFlag;
    uint32_t ppsDeblockingDisabledFlag;
    uint32_t chromaQpOffsetListEnabledFlag;
    uint32_t sliceActQpOffsetsPresentFlag;
    uint32_t sliceHeaderExtensionPresentFlag;
    bool     valid;

    PpsInfo() { memset(this, 0, sizeof(*this)); }
};

static uint32_t ceilLog2(uint32_t value)
{
    uint32_t bits = 0;
    while ((1u << bits) < value)
        bits++;
    return bits;
}

static void skipProfileTierLevel(BitReader& bits, uint32_t maxSubLayersMinus1)
{
    bits.u(2 + 1 + 5);                     /* space, tier, profile idc */
    bits.u(32);                            /* compatibility flags */
    bits.u(4);                             /* source/packing constraints */
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

static void skipScalingListData(BitReader& bits)
{
    for (uint32_t sizeId = 0; sizeId < 4 && !bits.error(); sizeId++)
    {
        uint32_t step = (sizeId == 3) ? 3 : 1;
        for (uint32_t matrixId = 0; matrixId < 6 && !bits.error(); matrixId += step)
        {
            if (!bits.u(1))
                bits.ue();
            else
            {
                uint32_t coefNum = X265_MIN(64u, 1u << (4 + (sizeId << 1)));
                if (sizeId > 1)
                    bits.se();
                for (uint32_t i = 0; i < coefNum && !bits.error(); i++)
                    bits.se();
            }
        }
    }
}

static bool parseSps(const std::vector<uint8_t>& rbsp, SpsInfo& sps)
{
    BitReader bits(rbsp.data(), rbsp.size(), 16);
    bits.u(4);                                        /* vps id */
    uint32_t maxSubLayersMinus1 = bits.u(3);
    bits.u(1);                                        /* temporal id nesting */
    skipProfileTierLevel(bits, maxSubLayersMinus1);
    bits.ue();                                        /* sps id */
    uint32_t chromaFormatIdc = bits.ue();
    uint32_t separateColourPlaneFlag = 0;
    if (chromaFormatIdc == 3)
        separateColourPlaneFlag = bits.u(1);
    uint32_t width = bits.ue();
    uint32_t height = bits.ue();
    if (bits.u(1))                                    /* conformance window */
    {
        bits.ue(); bits.ue(); bits.ue(); bits.ue();
    }
    bits.ue();                                        /* bit_depth_luma_minus8 */
    bits.ue();                                        /* bit_depth_chroma_minus8 */
    bits.ue();                                        /* log2_max_pic_order_cnt_lsb_minus4 */
    uint32_t subLayerOrdering = bits.u(1);
    for (uint32_t i = subLayerOrdering ? 0 : maxSubLayersMinus1;
         i <= maxSubLayersMinus1 && !bits.error(); i++)
    {
        bits.ue(); bits.ue(); bits.ue();
    }
    uint32_t log2MinCbMinus3 = bits.ue();
    uint32_t log2DiffMaxMinCb = bits.ue();
    bits.ue();                                        /* min transform size */
    bits.ue();                                        /* transform size range */
    bits.ue();                                        /* hierarchy depth inter */
    bits.ue();                                        /* hierarchy depth intra */
    if (bits.u(1) && bits.u(1))                       /* scaling list present */
        skipScalingListData(bits);
    bits.u(1);                                        /* amp_enabled_flag */
    sps.saoEnabledFlag = bits.u(1);

    if (bits.error())
        return false;

    uint32_t ctbLog2Size = log2MinCbMinus3 + 3 + log2DiffMaxMinCb;
    if (ctbLog2Size < 4 || ctbLog2Size > 6 || !width || !height)
        return false;
    uint32_t ctbSize = 1u << ctbLog2Size;
    uint32_t widthInCtbs = (width + ctbSize - 1) / ctbSize;
    uint32_t heightInCtbs = (height + ctbSize - 1) / ctbSize;

    sps.picSizeInCtbs = widthInCtbs * heightInCtbs;
    sps.addressBits = ceilLog2(sps.picSizeInCtbs);
    sps.separateColourPlaneFlag = separateColourPlaneFlag;
    sps.chromaArrayType = separateColourPlaneFlag ? 0 : chromaFormatIdc;
    sps.valid = true;
    return true;
}

static bool parsePps(const std::vector<uint8_t>& rbsp, PpsInfo& pps)
{
    BitReader bits(rbsp.data(), rbsp.size(), 16);
    bits.ue();                                        /* pps id */
    bits.ue();                                        /* sps id */
    pps.dependentSlicesEnabledFlag = bits.u(1);
    pps.outputFlagPresentFlag = bits.u(1);
    pps.numExtraSliceHeaderBits = bits.u(3);
    bits.u(1);                                        /* sign data hiding */
    bits.u(1);                                        /* cabac init present */
    bits.ue();                                        /* num_ref_idx_l0_default */
    bits.ue();                                        /* num_ref_idx_l1_default */
    bits.se();                                        /* init_qp_minus26 */
    bits.u(1);                                        /* constrained intra pred */
    uint32_t transformSkipEnabled = bits.u(1);
    if (bits.u(1))                                    /* cu_qp_delta_enabled */
        bits.ue();
    bits.se();                                        /* pps_cb_qp_offset */
    bits.se();                                        /* pps_cr_qp_offset */
    pps.sliceChromaQpOffsetsPresentFlag = bits.u(1);
    bits.u(1);                                        /* weighted_pred_flag */
    bits.u(1);                                        /* weighted_bipred_flag */
    bits.u(1);                                        /* transquant_bypass */
    pps.tilesEnabledFlag = bits.u(1);
    pps.entropyCodingSyncEnabledFlag = bits.u(1);
    if (pps.tilesEnabledFlag)
    {
        uint32_t columns = bits.ue();
        uint32_t rows = bits.ue();
        if (!bits.u(1))                               /* uniform_spacing_flag */
        {
            for (uint32_t i = 0; i < columns && !bits.error(); i++)
                bits.ue();
            for (uint32_t i = 0; i < rows && !bits.error(); i++)
                bits.ue();
        }
        bits.u(1);                                    /* loop filter across tiles */
    }
    pps.loopFilterAcrossSlicesEnabledFlag = bits.u(1);
    if (bits.u(1))                                    /* deblocking control present */
    {
        pps.deblockingOverrideEnabledFlag = bits.u(1);
        pps.ppsDeblockingDisabledFlag = bits.u(1);
        if (!pps.ppsDeblockingDisabledFlag)
        {
            bits.se();
            bits.se();
        }
    }
    if (bits.u(1))                                    /* pps scaling list present */
        skipScalingListData(bits);
    bits.u(1);                                        /* lists_modification_present */
    bits.ue();                                        /* log2_parallel_merge_level */
    pps.sliceHeaderExtensionPresentFlag = bits.u(1);
    if (bits.u(1))                                    /* pps_extension_present_flag */
    {
        uint32_t rangeExtension = bits.u(1);
        uint32_t multilayerExtension = bits.u(1);
        uint32_t extension3d = bits.u(1);
        uint32_t sccExtension = bits.u(1);
        uint32_t extension4bits = bits.u(4);
        if (multilayerExtension || extension3d || extension4bits)
            return false;                             /* not modelled here */
        if (rangeExtension)
        {
            if (transformSkipEnabled)
                bits.ue();
            bits.u(1);                                /* cross component pred */
            pps.chromaQpOffsetListEnabledFlag = bits.u(1);
            if (pps.chromaQpOffsetListEnabledFlag)
            {
                bits.ue();                            /* diff_cu_chroma_qp_offset_depth */
                uint32_t listLen = bits.ue() + 1;
                for (uint32_t i = 0; i < listLen && !bits.error(); i++)
                {
                    bits.se();
                    bits.se();
                }
            }
            bits.ue();                                /* log2_sao_offset_scale_luma */
            bits.ue();                                /* log2_sao_offset_scale_chroma */
        }
        if (sccExtension)
        {
            bits.u(1);                                /* curr_pic_ref_enabled */
            if (bits.u(1))                            /* adaptive colour transform */
            {
                pps.sliceActQpOffsetsPresentFlag = bits.u(1);
                bits.se();
                bits.se();
                bits.se();
            }
            if (bits.u(1))                            /* palette predictor init */
                return false;                         /* not modelled here */
        }
    }

    if (bits.error())
        return false;
    pps.valid = true;
    return true;
}

/* ---------------------------------------------------------- slice analysis */

struct SliceSegment
{
    uint32_t address;
    uint32_t headerBytes;
    int64_t  dataBytes;
};

/* Returns the size in bytes of slice_segment_header(), byte_alignment()
 * included, or -1 when the header cannot be delimited. */
static int64_t sliceHeaderBytes(const std::vector<uint8_t>& rbsp, uint32_t nalType,
                                const SpsInfo& sps, const PpsInfo& pps,
                                uint32_t& address)
{
    BitReader bits(rbsp.data(), rbsp.size(), 16);
    uint32_t firstSlice = bits.u(1);
    if (nalType >= 16 && nalType <= 23)
        bits.u(1);                                    /* no_output_of_prior_pics */
    bits.ue();                                        /* slice_pic_parameter_set_id */

    uint32_t dependent = 0;
    address = 0;
    if (!firstSlice)
    {
        if (pps.dependentSlicesEnabledFlag)
            dependent = bits.u(1);
        address = bits.u(sps.addressBits);
    }

    uint32_t saoLuma = 0, saoChroma = 0;
    uint32_t deblockingDisabled = pps.ppsDeblockingDisabledFlag;
    if (!dependent)
    {
        for (uint32_t i = 0; i < pps.numExtraSliceHeaderBits; i++)
            bits.u(1);
        uint32_t sliceType = bits.ue();
        if (pps.outputFlagPresentFlag)
            bits.u(1);
        if (sps.separateColourPlaneFlag == 1)
            bits.u(2);
        if (nalType != 19 && nalType != 20)
            return -1;                                /* only IDR is modelled */
        if (sps.saoEnabledFlag)
        {
            saoLuma = bits.u(1);
            if (sps.chromaArrayType)
                saoChroma = bits.u(1);
        }
        if (sliceType != 2)
            return -1;                                /* only I slices are modelled */
        bits.se();                                    /* slice_qp_delta */
        if (pps.sliceChromaQpOffsetsPresentFlag)
        {
            bits.se();
            bits.se();
        }
        if (pps.sliceActQpOffsetsPresentFlag)
        {
            bits.se();
            bits.se();
            bits.se();
        }
        if (pps.chromaQpOffsetListEnabledFlag)
            bits.u(1);
        uint32_t override = 0;
        if (pps.deblockingOverrideEnabledFlag)
            override = bits.u(1);
        if (override)
        {
            deblockingDisabled = bits.u(1);
            if (!deblockingDisabled)
            {
                bits.se();
                bits.se();
            }
        }
        if (pps.loopFilterAcrossSlicesEnabledFlag &&
            (saoLuma || saoChroma || !deblockingDisabled))
            bits.u(1);
    }

    if (pps.tilesEnabledFlag || pps.entropyCodingSyncEnabledFlag)
    {
        uint32_t entryPoints = bits.ue();
        if (entryPoints)
        {
            uint32_t offsetLen = bits.ue() + 1;
            for (uint32_t i = 0; i < entryPoints && !bits.error(); i++)
                bits.u(offsetLen);
        }
    }
    if (pps.sliceHeaderExtensionPresentFlag)
    {
        uint32_t length = bits.ue();
        for (uint32_t i = 0; i < length && !bits.error(); i++)
            bits.u(8);
    }

    /* byte_alignment() */
    if (bits.u(1) != 1)
        return -1;
    while (!bits.error() && !bits.byteAligned())
        if (bits.u(1))
            return -1;
    if (bits.error())
        return -1;

    return (int64_t)((bits.position() - 16) / 8);
}

/* Splits an Annex-B stream and measures every emitted slice segment. Returns
 * false when the stream cannot be analysed at all. */
static bool analyseStream(const std::vector<uint8_t>& stream,
                          std::vector<SliceSegment>& slices,
                          const char*& failure)
{
    slices.clear();
    SpsInfo sps;
    PpsInfo pps;
    std::vector<size_t> starts;

    for (size_t i = 0; i + 2 < stream.size(); i++)
        if (!stream[i] && !stream[i + 1] && stream[i + 2] == 1)
        {
            starts.push_back(i + 3);
            i += 2;
        }
    if (starts.empty())
    {
        failure = "no Annex-B start code";
        return false;
    }

    std::vector<uint8_t> rbsp;
    for (size_t n = 0; n < starts.size(); n++)
    {
        size_t begin = starts[n];
        size_t end = (n + 1 < starts.size()) ? starts[n + 1] - 3 : stream.size();
        while (end > begin && !stream[end - 1])
            end--;                                    /* trailing_zero_8bits */
        if (end < begin + 2)
        {
            failure = "truncated NAL unit";
            return false;
        }

        extractRbsp(&stream[begin], end - begin, rbsp);
        uint32_t nalType = (rbsp[0] >> 1) & 0x3F;
        if (nalType == 33)
        {
            if (!parseSps(rbsp, sps))
            {
                failure = "unsupported SPS";
                return false;
            }
        }
        else if (nalType == 34)
        {
            if (!parsePps(rbsp, pps))
            {
                failure = "unsupported PPS";
                return false;
            }
        }
        else if (nalType <= 21)
        {
            if (!sps.valid || !pps.valid)
            {
                failure = "slice segment before its parameter sets";
                return false;
            }
            SliceSegment segment;
            int64_t headerBytes = sliceHeaderBytes(rbsp, nalType, sps, pps,
                                                   segment.address);
            if (headerBytes < 0)
            {
                failure = "slice segment header could not be delimited";
                return false;
            }
            segment.headerBytes = (uint32_t)headerBytes;
            segment.dataBytes = (int64_t)(rbsp.size() - 2) - headerBytes;
            slices.push_back(segment);
        }
    }
    return true;
}

/* ----------------------------------------------------------------- encoding */

struct Case
{
    const char* name;
    int         width;
    int         height;
    int         slices;
    int         wpp;
    const char* pools;
    const char* intent;
};

static void appendNals(std::vector<uint8_t>& stream, const x265_nal* nals,
                       uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
        stream.insert(stream.end(), nals[i].payload,
                      nals[i].payload + nals[i].sizeBytes);
}

/* Encodes one zero-valued frame. Returns false only when the encoder refuses
 * the configuration, which is a conforming outcome and leaves stream empty. */
static bool encodeOneFrame(const Case& test, std::vector<uint8_t>& stream,
                           const char*& failure)
{
    stream.clear();

    x265_param* param = x265_param_alloc();
    if (!param)
    {
        failure = "x265_param_alloc failed";
        return false;
    }
    x265_param_default(param);
    param->sourceWidth = test.width;
    param->sourceHeight = test.height;
    param->internalCsp = X265_CSP_I420;
    param->internalBitDepth = X265_DEPTH;
    param->fpsNum = 30;
    param->fpsDenom = 1;
    param->maxSlices = test.slices;
    param->bEnableWavefront = test.wpp;
    param->bRepeatHeaders = 1;
    param->logLevel = X265_LOG_WARNING;
    if (test.pools)
        x265_param_parse(param, "pools", test.pools);

    x265_encoder* encoder = x265_encoder_open(param);
    if (!encoder)
    {
        x265_param_free(param);
        failure = "configuration declined by x265_encoder_open";
        return false;
    }

    const size_t pixelBytes = (X265_DEPTH > 8) ? 2 : 1;
    const size_t lumaBytes = (size_t)test.width * test.height * pixelBytes;
    const size_t chromaBytes = lumaBytes / 4;
    std::vector<uint8_t> luma(lumaBytes, 0);
    std::vector<uint8_t> cb(chromaBytes, 0);
    std::vector<uint8_t> cr(chromaBytes, 0);

    x265_picture picture;
    x265_picture_init(param, &picture);
    picture.planes[0] = luma.data();
    picture.planes[1] = cb.data();
    picture.planes[2] = cr.data();
    picture.stride[0] = (int)(test.width * pixelBytes);
    picture.stride[1] = (int)(test.width * pixelBytes / 2);
    picture.stride[2] = picture.stride[1];

    bool ok = true;
    x265_nal* nals = NULL;
    uint32_t numNal = 0;
    if (x265_encoder_encode(encoder, &nals, &numNal, &picture, NULL) < 0)
    {
        failure = "x265_encoder_encode failed";
        ok = false;
    }
    appendNals(stream, nals, numNal);
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
        if (!ret)
            break;
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
        /* Post-validation WPP disablement: fewer than three CTU columns. */
        { "trigger-few-columns", 128, 128, 2, 1, NULL,
          "WPP disabled after the multiple-slice request was accepted" },
        /* Post-validation WPP disablement: no thread pool remains. */
        { "trigger-no-thread-pool", 192, 128, 2, 1, "none",
          "WPP disabled after the multiple-slice request was accepted" },
        /* Neighbouring valid behaviour that must keep working. */
        { "control-wpp-two-slices", 192, 128, 2, 1, NULL,
          "supported multiple-slice configuration with WPP retained" },
        { "control-single-slice", 192, 128, 1, 0, NULL,
          "supported single-slice configuration without WPP" },
    };

    int failures = 0;
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        const Case& test = cases[i];
        std::vector<uint8_t> stream;
        std::vector<SliceSegment> slices;
        const char* failure = NULL;
        bool passed;
        char detail[256];

        if (!encodeOneFrame(test, stream, failure))
        {
            /* Declining the configuration emits no slice segment at all, so
             * the invariant holds. Any other encoder failure is a test error. */
            passed = !strcmp(failure, "configuration declined by x265_encoder_open");
            snprintf(detail, sizeof(detail), "%s", failure);
        }
        else if (!analyseStream(stream, slices, failure))
        {
            passed = false;
            snprintf(detail, sizeof(detail), "%s", failure);
        }
        else if (slices.empty())
        {
            passed = false;
            snprintf(detail, sizeof(detail), "no slice segment emitted");
        }
        else
        {
            passed = true;
            int written = snprintf(detail, sizeof(detail), "%d segment(s):",
                                   (int)slices.size());
            for (size_t s = 0; s < slices.size(); s++)
            {
                if (slices[s].dataBytes < 1)
                    passed = false;
                if (written >= 0 && written < (int)sizeof(detail))
                    written += snprintf(detail + written, sizeof(detail) - written,
                                        " [addr=%u header=%u data=%lld]",
                                        slices[s].address, slices[s].headerBytes,
                                        (long long)slices[s].dataBytes);
            }
        }

        if (!passed)
            failures++;
        printf("slice segment data present, %s (%dx%d slices=%d wpp=%d pools=%s, "
               "%d-bit): %s [%s]\n",
               test.name, test.width, test.height, test.slices, test.wpp,
               test.pools ? test.pools : "default", X265_DEPTH,
               passed ? "PASS" : "FAIL", detail);
    }

    printf("slice segment structure (%d-bit): %s [%d case(s) failed]\n",
           X265_DEPTH, failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
