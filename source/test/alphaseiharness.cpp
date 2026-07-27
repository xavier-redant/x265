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

#include "common.h"
#include "bitstream.h"
#include "nal.h"
#include "sei.h"

#include <stdint.h>
#include <stdio.h>
#include <vector>

using namespace X265_NS;

class BitReader
{
public:
    BitReader(const uint8_t* data, uint32_t size)
        : m_data(data)
        , m_size(size)
        , m_position(0)
    {}

    bool read(uint32_t width, uint32_t& value)
    {
        if (m_position + width > m_size * 8)
            return false;

        value = 0;
        for (uint32_t i = 0; i < width; i++)
        {
            value <<= 1;
            value |= (m_data[m_position >> 3] >> (7 - (m_position & 7))) & 1;
            m_position++;
        }
        return true;
    }

private:
    const uint8_t* m_data;
    uint32_t       m_size;
    uint32_t       m_position;
};

static bool decodePayload(const x265_nal& nal, std::vector<uint8_t>& rbsp)
{
    uint32_t offset;
    if (nal.sizeBytes >= 6 && !nal.payload[0] && !nal.payload[1] &&
        !nal.payload[2] && nal.payload[3] == 1)
        offset = 4;
    else if (nal.sizeBytes >= 5 && !nal.payload[0] && !nal.payload[1] &&
             nal.payload[2] == 1)
        offset = 3;
    else
        return false;

    offset += 2;
    uint32_t zeroCount = 0;
    for (; offset < nal.sizeBytes; offset++)
    {
        uint8_t value = nal.payload[offset];
        if (zeroCount >= 2 && value == 3)
        {
            zeroCount = 0;
            continue;
        }

        rbsp.push_back(value);
        zeroCount = value ? 0 : zeroCount + 1;
    }
    return true;
}

int main()
{
    SPS sps;
    Bitstream bitstream;
    NALList list;
    SEIAlphaChannelInfo alpha;
    alpha.alpha_channel_cancel_flag = false;
    alpha.writeSEImessages(bitstream, sps, NAL_UNIT_PREFIX_SEI, list, false, 1);

    if (list.m_numNal != 1)
    {
        printf("alpha SEI NAL count: FAIL\n");
        return 1;
    }

    std::vector<uint8_t> rbsp;
    if (!decodePayload(list.m_nal[0], rbsp) || rbsp.size() < 3)
    {
        printf("alpha SEI payload extraction: FAIL\n");
        return 1;
    }

    uint32_t offset = 0;
    uint32_t payloadType = 0;
    while (offset < rbsp.size() && rbsp[offset] == 0xFF)
    {
        payloadType += 0xFF;
        offset++;
    }
    if (offset >= rbsp.size())
        return 1;
    payloadType += rbsp[offset++];

    uint32_t payloadSize = 0;
    while (offset < rbsp.size() && rbsp[offset] == 0xFF)
    {
        payloadSize += 0xFF;
        offset++;
    }
    if (offset >= rbsp.size())
        return 1;
    payloadSize += rbsp[offset++];

    if (payloadType != ALPHA_CHANNEL_INFO || offset + payloadSize > rbsp.size())
    {
        printf("alpha SEI header: FAIL\n");
        return 1;
    }

    BitReader bits(&rbsp[offset], payloadSize);
    uint32_t cancelFlag;
    uint32_t useIdc;
    uint32_t depthMinus8;
    uint32_t transparentValue;
    uint32_t opaqueValue;
    uint32_t incrFlag;
    uint32_t clipFlag;
    uint32_t expectedDepthMinus8 = X265_DEPTH - 8;
    uint32_t valueWidth = expectedDepthMinus8 + 9;
    uint32_t expectedOpaqueValue = PIXEL_MAX;

    bool parsed =
        bits.read(1, cancelFlag) &&
        bits.read(3, useIdc) &&
        bits.read(3, depthMinus8) &&
        bits.read(valueWidth, transparentValue) &&
        bits.read(valueWidth, opaqueValue) &&
        bits.read(1, incrFlag) &&
        bits.read(1, clipFlag);

    bool passed =
        parsed &&
        cancelFlag == 0 &&
        useIdc == 0 &&
        depthMinus8 == expectedDepthMinus8 &&
        transparentValue == 0 &&
        opaqueValue == expectedOpaqueValue &&
        incrFlag == 0 &&
        clipFlag == 0;

    printf(
        "alpha SEI depth/value widths (%d-bit): %s"
        " [depth=%u opaque=%u]\n",
        X265_DEPTH,
        passed ? "PASS" : "FAIL",
        depthMinus8,
        opaqueValue);
    return passed ? 0 : 1;
}
