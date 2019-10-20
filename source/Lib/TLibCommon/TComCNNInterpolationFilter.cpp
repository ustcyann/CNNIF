/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2015, ITU/ISO/IEC
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
*    be used to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
* \file
* \brief Implementation of TComCNNInterpolationFilter class
*/

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "TComRom.h"
#include "TComCNNInterpolationFilter.h"
#include <assert.h>


//! \ingroup TLibCommon
//! \{

Void TComCNNInterpolationFilter::filterCNN( const ComponentID compID, Pel *src, Int srcStride, Pel *dst, Int dstStride, 
                                            Int width, Int height, RefPicList eRefList,
                                            Int yFrac, Int xFrac,  Int iQP, Bool isLast, 
                                            const ChromaFormat fmt, const Int bitDepth )
{
  if ( yFrac == 0 && xFrac == 0 )  //integer pel position
  {
    m_DCTIF.filterHor(compID, src, srcStride, dst, dstStride, width, height, 0, isLast, fmt, bitDepth);
    return;
  }

  Int iPadding  = 2 * PADDING_SIZE;
  Int iPaddingH = PADDING_SIZE;

  Int inputBlkWidth  = width + iPadding;
  Int inputBlkHeight = height + iPadding;

  TComConvNet IFConvNet;
  string netFile = IFConvNet.getCNNIFNetFile(yFrac, xFrac, iQP);
  string caffeModel = IFConvNet.getCNNIFCaffeModel(yFrac, xFrac, iQP, eRefList);
  IFConvNet.creat(netFile, caffeModel, "rec", "CPU", bitDepth);
  cv::Mat inputBlkMat(inputBlkHeight, inputBlkWidth, CV_32FC1);
  IFConvNet.setInutBlkSize(inputBlkHeight, inputBlkWidth);
  IFConvNet.setInputBlock(src - iPaddingH*srcStride - iPaddingH, srcStride, inputBlkMat, inputBlkHeight, inputBlkWidth);
  IFConvNet.ConvNetForward(inputBlkMat);
  std::vector<Float>result = IFConvNet.getConvNetOutputCenter(iPaddingH);
  IFConvNet.copyFromVectorToPointer(result, dst, dstStride, height, width, bitDepth, isLast);
  IFConvNet.destroy(netFile, caffeModel, "CPU");
}

Void TComCNNInterpolationFilter::getCNNFracInterpolation( Pel* src, Int srcStride, Pel* dst, Int dstStride, Int width, Int height, Int bitDepth, Bool isLast )
{
  Int shift = IF_INTERNAL_PREC - bitDepth;
  Int maxVal = (1 << bitDepth) - 1;

  if (isLast)
  {
    Pel temp;
    for (Int y = 0; y < height; y++)
    {
      for (Int x = 0; x < width; x++)
      {
        temp = ((src[x] + IF_INTERNAL_OFFS) >> shift); 
        temp = Clip3<Pel>(0, maxVal, temp);
        dst[x] = temp;
      }
      src += srcStride;
      dst += dstStride;
    }
  }
  else
  {
    for (Int y = 0; y < height; y++)
    {
      memcpy(dst, src, sizeof(Pel)*width);
      src += srcStride;
      dst += dstStride;
    }
  }
}