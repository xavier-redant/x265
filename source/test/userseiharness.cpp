/*****************************************************************************
 * Copyright (C) 2013-2026 MulticoreWare, Inc
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
 *****************************************************************************/

#include "x265.h"

#include <stdint.h>
#include <stdio.h>
#include <vector>

enum UserSEIInput
{
    NORMAL_INPUT,
    NEGATIVE_PAYLOAD_COUNT,
    NULL_PAYLOAD_ARRAY,
    NULL_PAYLOAD_DATA
};

static bool runCase(const char *name, const std::vector<uint8_t> &seiBytes, bool expectReject,
                    UserSEIInput input = NORMAL_INPUT)
{
    const int width = 64;
    const int height = 64;
    x265_param *param = x265_param_alloc();
    if (!param)
        return false;
    if (x265_param_default_preset(param, "ultrafast", "zerolatency") < 0)
    {
        x265_param_free(param);
        return false;
    }

    param->sourceWidth = width;
    param->sourceHeight = height;
    param->fpsNum = 1;
    param->fpsDenom = 1;
    param->totalFrames = 1;
    param->internalCsp = X265_CSP_I420;
    param->bframes = 0;
    param->keyframeMax = 1;
    param->keyframeMin = 1;
    param->logLevel = X265_LOG_ERROR;

    x265_encoder *encoder = x265_encoder_open(param);
    if (!encoder)
    {
        x265_param_free(param);
        return false;
    }

    x265_picture picture;
    x265_picture_init(param, &picture);

    std::vector<uint8_t> y(width * height, 16);
    std::vector<uint8_t> u((width / 2) * (height / 2), 128);
    std::vector<uint8_t> v((width / 2) * (height / 2), 128);
    picture.planes[0] = &y[0];
    picture.planes[1] = &u[0];
    picture.planes[2] = &v[0];
    picture.stride[0] = width;
    picture.stride[1] = width / 2;
    picture.stride[2] = width / 2;

    x265_sei_payload payload;
    payload.payloadSize = (int)seiBytes.size();
    payload.payloadType = USER_DATA_REGISTERED_ITU_T_T35;
    payload.payload = input == NULL_PAYLOAD_DATA || seiBytes.empty() ? NULL : const_cast<uint8_t *>(&seiBytes[0]);
    picture.userSEI.numPayloads = input == NEGATIVE_PAYLOAD_COUNT ? -1 : 1;
    picture.userSEI.payloads = input == NULL_PAYLOAD_ARRAY ? NULL : &payload;

    x265_nal *nals = NULL;
    uint32_t nalCount = 0;
    int result = x265_encoder_encode(encoder, &nals, &nalCount, &picture, NULL);
    bool passed = expectReject ? result < 0 : result >= 0;
    int picturesOutput = result > 0 ? result : 0;

    if (!expectReject && result >= 0)
    {
        while (true)
        {
            result = x265_encoder_encode(encoder, &nals, &nalCount, NULL, NULL);
            if (result < 0)
            {
                passed = false;
                break;
            }
            picturesOutput += result;
            if (!result)
                break;
        }
        passed = passed && picturesOutput == 1;
    }

    x265_encoder_close(encoder);
    x265_param_free(param);

    printf("%s: %s\n", name, passed ? "PASS" : "FAIL");
    return passed;
}

int main()
{
    bool passed = true;
    passed &=
        runCase("negative payload count is rejected", std::vector<uint8_t>(2, 0x00), true, NEGATIVE_PAYLOAD_COUNT);
    passed &= runCase("NULL payload array is rejected", std::vector<uint8_t>(2, 0x00), true, NULL_PAYLOAD_ARRAY);
    passed &= runCase("NULL T.35 payload is rejected", std::vector<uint8_t>(2, 0x00), true, NULL_PAYLOAD_DATA);
    passed &= runCase("one-byte non-FF payload is rejected", std::vector<uint8_t>(1, 0xB5), true);

    std::vector<uint8_t> shortFF;
    shortFF.push_back(0xFF);
    shortFF.push_back(0x00);
    passed &= runCase("two-byte FF payload is rejected", shortFF, true);

    std::vector<uint8_t> validNonFF;
    validNonFF.push_back(0xB5);
    validNonFF.push_back(0x00);
    passed &= runCase("two-byte non-FF payload is accepted", validNonFF, false);

    std::vector<uint8_t> validFF;
    validFF.push_back(0xFF);
    validFF.push_back(0x00);
    validFF.push_back(0x00);
    passed &= runCase("three-byte FF payload is accepted", validFF, false);

    x265_cleanup();
    return passed ? 0 : 1;
}
