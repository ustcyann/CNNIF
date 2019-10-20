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

/** \file     TComPic.cpp
    \brief    picture class
*/

#include "TComPic.h"
#include "SEI.h"
#if CNN_FRAC_INTERPOLATION
#include "TComConvNet.h"
#include "TComInterpolationFilter.h"
#endif

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComPic::TComPic()
: m_uiTLayer                              (0)
, m_bUsedByCurr                           (false)
, m_bIsLongTerm                           (false)
, m_pcPicYuvPred                          (NULL)

#if OUTPUT_PREDICTION
, m_pcPicYuvPredTmp                       (NULL)
#endif

, m_pcPicYuvResi                          (NULL)
, m_bReconstructed                        (false)
, m_bNeededForOutput                      (false)
, m_uiCurrSliceIdx                        (0)
, m_bCheckLTMSB                           (false)
{
  for(UInt i=0; i<NUM_PIC_YUV; i++)
  {
    m_apcPicYuv[i]      = NULL;
  }


#if CNN_FRAC_INTERPOLATION
  for (Int y = 0; y < 4; y++)
  {
    for (Int x = 0; x < 4; x++)
    {
      m_pcPicYuvFracInterpolation[y][x]=NULL;
    }
  }
#endif

#if ENABLE_BI_MODEL
  for (Int y = 0; y < 4; y++)
  {
    for (Int x = 0; x < 4; x++)
    {
      m_pcPicYuvFracInterpolationList1[y][x]=NULL;
    }
  }
#endif

#if OUTPUT_DCTIF_INTERPOLATED_FRAMES
  for (Int y = 0; y < 4; y++)
  {
    for (Int x = 0; x < 4; x++)
    {
      m_pcPicYuvDCTIF[y][x] = NULL;
    }
  }
#endif

#if DRAW_PARTITION_MODE
  m_apcPicYuvRecTmp = NULL;
#endif

}

TComPic::~TComPic()
{
}

Void TComPic::create( const TComSPS &sps, const TComPPS &pps, const Bool bIsVirtual)
{
  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();
  const Int          iWidth          = sps.getPicWidthInLumaSamples();
  const Int          iHeight         = sps.getPicHeightInLumaSamples();
  const UInt         uiMaxCuWidth    = sps.getMaxCUWidth();
  const UInt         uiMaxCuHeight   = sps.getMaxCUHeight();
  const UInt         uiMaxDepth      = sps.getMaxTotalCUDepth();

  m_picSym.create( sps, pps, uiMaxDepth );
  if (!bIsVirtual)
  {
    m_apcPicYuv[PIC_YUV_ORG    ]   = new TComPicYuv;  m_apcPicYuv[PIC_YUV_ORG     ]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
    m_apcPicYuv[PIC_YUV_TRUE_ORG]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_TRUE_ORG]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );
  }
  m_apcPicYuv[PIC_YUV_REC]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_REC]->create( iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true );

#if OUTPUT_PREDICTION
  m_pcPicYuvPredTmp = new TComPicYuv;  m_pcPicYuvPredTmp->create(iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true);
#endif

#if CNN_FRAC_INTERPOLATION
  for (Int y = 0; y < 4; y++)
  {
    for (Int x = 0; x < 4; x++)
    {
      if (y == 0 && x == 0)
      {
        continue;
      }
      m_pcPicYuvFracInterpolation[y][x] = new TComPicYuv;
      m_pcPicYuvFracInterpolation[y][x]->create(iWidth, iHeight, CHROMA_400, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true);
    }
  }
#endif

#if ENABLE_BI_MODEL
  for (Int y = 0; y < 4; y++)
  {
    for (Int x = 0; x < 4; x++)
    {
      if (y == 0 && x == 0)
      {
        continue;
      }
      m_pcPicYuvFracInterpolationList1[y][x] = new TComPicYuv;
      m_pcPicYuvFracInterpolationList1[y][x]->create(iWidth, iHeight, CHROMA_400, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true);
    }
  }
#endif

#if OUTPUT_DCTIF_INTERPOLATED_FRAMES
  for (Int y = 0; y < 4; y++)
  {
    for (Int x = 0; x < 4; x++)
    {
      m_pcPicYuvDCTIF[y][x]=new TComPicYuv;
      m_pcPicYuvDCTIF[y][x]->create(iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true);
    }
  }
#endif

#if DRAW_PARTITION_MODE
  m_apcPicYuvRecTmp = new TComPicYuv; m_apcPicYuvRecTmp->create(iWidth, iHeight, chromaFormatIDC, uiMaxCuWidth, uiMaxCuHeight, uiMaxDepth, true);
#endif


  // there are no SEI messages associated with this picture initially
  if (m_SEIs.size() > 0)
  {
    deleteSEIs (m_SEIs);
  }
  m_bUsedByCurr = false;
}

Void TComPic::destroy()
{
  m_picSym.destroy();

  for(UInt i=0; i<NUM_PIC_YUV; i++)
  {
    if (m_apcPicYuv[i])
    {
      m_apcPicYuv[i]->destroy();
      delete m_apcPicYuv[i];
      m_apcPicYuv[i]  = NULL;
    }
  }

#if OUTPUT_PREDICTION
  if (m_pcPicYuvPredTmp)
  {
    m_pcPicYuvPredTmp->destroy();
    delete m_pcPicYuvPredTmp;
    m_pcPicYuvPredTmp = NULL;
  }
#endif


#if CNN_FRAC_INTERPOLATION
  for (Int y = 0; y < 4; y++)
  {
    for (Int x = 0; x < 4; x++)
    {
      if (m_pcPicYuvFracInterpolation[y][x])
      {
        m_pcPicYuvFracInterpolation[y][x]->destroy();
        delete m_pcPicYuvFracInterpolation[y][x];
        m_pcPicYuvFracInterpolation[y][x] = NULL;
      }
    }
  }
#endif

#if ENABLE_BI_MODEL
  for (Int y = 0; y < 4; y++)
  {
    for (Int x = 0; x < 4; x++)
    {
      if (m_pcPicYuvFracInterpolationList1[y][x])
      {
        m_pcPicYuvFracInterpolationList1[y][x]->destroy();
        delete m_pcPicYuvFracInterpolationList1[y][x];
        m_pcPicYuvFracInterpolationList1[y][x] = NULL;
      }
    }
  }
#endif

#if OUTPUT_DCTIF_INTERPOLATED_FRAMES
  for (Int y = 0; y < 4; y++)
  {
    for (Int x = 0; x < 4; x++)
    {
      m_pcPicYuvDCTIF[y][x]->destroy();
      delete m_pcPicYuvDCTIF[y][x];
      m_pcPicYuvDCTIF[y][x] = NULL;
    }
  }
#endif

#if DRAW_PARTITION_MODE
  if (m_apcPicYuvRecTmp)
  {
    m_apcPicYuvRecTmp->destroy();
    delete m_apcPicYuvRecTmp;
    m_apcPicYuvRecTmp = NULL;
  }
#endif


  deleteSEIs(m_SEIs);
}

Void TComPic::compressMotion()
{
  TComPicSym* pPicSym = getPicSym();
  for ( UInt uiCUAddr = 0; uiCUAddr < pPicSym->getNumberOfCtusInFrame(); uiCUAddr++ )
  {
    TComDataCU* pCtu = pPicSym->getCtu(uiCUAddr);
    pCtu->compressMV();
  }
}

Bool  TComPic::getSAOMergeAvailability(Int currAddr, Int mergeAddr)
{
  Bool mergeCtbInSliceSeg = (mergeAddr >= getPicSym()->getCtuTsToRsAddrMap(getCtu(currAddr)->getSlice()->getSliceCurStartCtuTsAddr()));
  Bool mergeCtbInTile     = (getPicSym()->getTileIdxMap(mergeAddr) == getPicSym()->getTileIdxMap(currAddr));
  return (mergeCtbInSliceSeg && mergeCtbInTile);
}

UInt TComPic::getSubstreamForCtuAddr(const UInt ctuAddr, const Bool bAddressInRaster, TComSlice *pcSlice)
{
  UInt subStrm;
  const bool bWPPEnabled=pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();
  const TComPicSym &picSym            = *(getPicSym());

  if ((bWPPEnabled && picSym.getFrameHeightInCtus()>1) || (picSym.getNumTiles()>1)) // wavefronts, and possibly tiles being used.
  {
    if (bWPPEnabled)
    {
      const UInt ctuRsAddr                = bAddressInRaster?ctuAddr : picSym.getCtuTsToRsAddrMap(ctuAddr);
      const UInt frameWidthInCtus         = picSym.getFrameWidthInCtus();
      const UInt tileIndex                = picSym.getTileIdxMap(ctuRsAddr);
      const UInt numTileColumns           = (picSym.getNumTileColumnsMinus1()+1);
      const TComTile *pTile               = picSym.getTComTile(tileIndex);
      const UInt firstCtuRsAddrOfTile     = pTile->getFirstCtuRsAddr();
      const UInt tileYInCtus              = firstCtuRsAddrOfTile / frameWidthInCtus;
      // independent tiles => substreams are "per tile"
      const UInt ctuLine                  = ctuRsAddr / frameWidthInCtus;
      const UInt startingSubstreamForTile =(tileYInCtus*numTileColumns) + (pTile->getTileHeightInCtus()*(tileIndex%numTileColumns));
      subStrm = startingSubstreamForTile + (ctuLine - tileYInCtus);
    }
    else
    {
      const UInt ctuRsAddr                = bAddressInRaster?ctuAddr : picSym.getCtuTsToRsAddrMap(ctuAddr);
      const UInt tileIndex                = picSym.getTileIdxMap(ctuRsAddr);
      subStrm=tileIndex;
    }
  }
  else
  {
    // dependent tiles => substreams are "per frame".
    subStrm = 0;
  }
  return subStrm;
}

#if CNN_FRAC_INTERPOLATION
Void TComPic::performCNNBasedInterpolation(RefPicList eRefList)
{
  const Int iQP = getSlice(0)->getSliceQp();
  const Int bitDepth = getSlice(0)->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);

  Int iPicHeight = getPicYuvRec()->getHeight(COMPONENT_Y);
  Int iPicWidth  = getPicYuvRec()->getWidth(COMPONENT_Y);
  getPicYuvRec()->setBorderExtension(false);
  getPicYuvRec()->extendPicBorder();
  Pel* pSrc   = getPicYuvRec()->getAddr(COMPONENT_Y);
  Int  Stride = getPicYuvRec()->getStride(COMPONENT_Y);

  Pel* pImgFrac[4][4];

#if ENABLE_BI_MODEL
  if (eRefList == REF_PIC_LIST_0)
  {
    for (Int y = 0; y < 4; y++)
    {
      for (Int x = 0; x < 4; x++)
      {
        if (y == 0 && x == 0)
          continue;
        pImgFrac[y][x] = getPicYuvFracInterpolation(y, x)->getAddr(COMPONENT_Y);
      }
    }
  }
  else
  {
    for (Int y = 0; y < 4; y++)
    {
      for (Int x = 0; x < 4; x++)
      {
        if (y == 0 && x == 0)
          continue;
        pImgFrac[y][x] = getPicYuvFracInterpolationList1(y, x)->getAddr(COMPONENT_Y);
      }
    }
  }
#else
  for (Int y = 0; y < 4; y++)
  {
    for (Int x = 0; x < 4; x++)
    {
      if (y == 0 && x == 0)
        continue;
      pImgFrac[y][x] = getPicYuvFracInterpolation(y, x)->getAddr(COMPONENT_Y);
    }
  }
#endif


  //const Int OutBlkSize = 256;
  const Int OutBlkSize = 576;
  printf("patch size=%d\n", OutBlkSize);

  Int iPicWidthExt  = iPicWidth  + 2 * PADDING_SIZE;
  Int iPicHeightExt = iPicHeight + 2 * PADDING_SIZE;
  Int iNumBlockHor  = (iPicWidthExt  % OutBlkSize == 0) ? (iPicWidthExt  / OutBlkSize) : (iPicWidthExt  / OutBlkSize + 1);
  Int iNumBlockVer  = (iPicHeightExt % OutBlkSize == 0) ? (iPicHeightExt / OutBlkSize) : (iPicHeightExt / OutBlkSize + 1);
  Int iTotalNumBlk  = iNumBlockHor*iNumBlockVer;


  for (Int yFrac = 0; yFrac < 4; yFrac++)
  {
    for (Int xFrac = 0; xFrac < 4; xFrac++)
    {
      if (!g_bEnabledCNNIFFrac[yFrac][xFrac])
        continue;
      // construct the network
      TComConvNet IFConvNet;
      std::string netFile = IFConvNet.getCNNIFNetFile(yFrac, xFrac, iQP);
      std::string caffeModel = IFConvNet.getCNNIFCaffeModel(yFrac, xFrac, iQP, eRefList);
      IFConvNet.creat(netFile, caffeModel, "rec", "CPU", getSlice(0)->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
      // Process block by block
      for (Int iBlk = 0; iBlk < iTotalNumBlk; iBlk++)
      {
        Int iRow = iBlk / iNumBlockHor;
        Int iCol = iBlk % iNumBlockHor;
        Int iHorOffset = iCol*OutBlkSize;
        Int iVerOffset = iRow*OutBlkSize;
        // block size of the current block
        Int targetBlkHeight = (iPicHeightExt - iVerOffset) < OutBlkSize ? (iPicHeightExt - iVerOffset) : OutBlkSize;
        Int targetBlkWidth = (iPicWidthExt - iHorOffset) < OutBlkSize ? (iPicWidthExt - iHorOffset) : OutBlkSize;
        // block size of the input block
        Int inputBlkHeight = targetBlkHeight + 2 * PADDING_SIZE;
        Int inputBlkWidth = targetBlkWidth + 2 * PADDING_SIZE;
        // the input/output pointer
        Int iRefPadOffset = -(PADDING_SIZE*Stride + PADDING_SIZE);
        const Pel* pIntRef = pSrc + iVerOffset*Stride + iHorOffset;
        pIntRef += iRefPadOffset; // for input region padding
        pIntRef += iRefPadOffset; // for processing margin

        Pel* pDst = pImgFrac[yFrac][xFrac] + iVerOffset*Stride + iHorOffset;
        pDst += iRefPadOffset; // for processing margin
        // prepare net forward
        cv::Mat inputBlkMat(inputBlkHeight, inputBlkWidth, CV_32FC1);
        IFConvNet.setInutBlkSize(inputBlkHeight, inputBlkWidth); // set input block size
        IFConvNet.setInputBlock(pIntRef, Stride, inputBlkMat, inputBlkHeight, inputBlkWidth);
        IFConvNet.ConvNetForward(inputBlkMat); // net forward
        std::vector<Float> vOutputBlk = IFConvNet.getConvNetOutputCenter(PADDING_SIZE);
        // process the output
        IFConvNet.copyFromVectorToPointer(vOutputBlk, pDst, Stride, targetBlkHeight, targetBlkWidth, bitDepth, false);
      }
      // destroy the network

      IFConvNet.destroy(netFile, caffeModel, "CPU");
      getPicYuvFracInterpolation(yFrac, xFrac)->setBorderExtension(false);
      getPicYuvFracInterpolation(yFrac, xFrac)->extendPicBorderWithPadding(PADDING_SIZE);

    }
  }

#if OUTPUT_CNNIF_INTERPOLATED_FRAMES
  TComPicYuv* pPicYuv = new TComPicYuv;
  pPicYuv->create(iPicWidth, iPicHeight, CHROMA_420, MAX_CU_SIZE, MAX_CU_SIZE, MAX_CU_DEPTH, true);

  UChar p[] = { '0', '1', '2', '3' };
  BitDepths biDepth;
  biDepth.recon[0] = biDepth.recon[1] = 8;

  std::string pName = "TEST_CONV_PhXX_Block";
  if (eRefList == REF_PIC_LIST_0)
  {
    pName += "_L0.yuv";
  }
  else
  {
    pName += "_L1.yuv";
  }

  Pel* cbAddr = pPicYuv->getAddr(COMPONENT_Cb);
  Pel* crAddr = pPicYuv->getAddr(COMPONENT_Cr);
  Int iStrideC = pPicYuv->getStride(COMPONENT_Cb);
  Int iwidthC = iPicWidth >> 1;
  Int iheightC = iPicHeight >> 1;
  for (Int y = 0; y < iheightC; y++)
  {
    for (Int x = 0; x < iwidthC; x++)
    {
      cbAddr[y*iStrideC + x] = 128;
      crAddr[y*iStrideC + x] = 128;
    }
  }

  for (Int yFrac = 0; yFrac < 4; yFrac++)
  {
    for (Int xFrac = 0; xFrac < 4; xFrac++)
    {
      if (!g_bEnabledCNNIFFrac[yFrac][xFrac])
        continue;

      pName[12] = p[yFrac];
      pName[13] = p[xFrac];

      Pel* lumaAddr = pPicYuv->getAddr(COMPONENT_Y);
      Int iStride = pPicYuv->getStride(COMPONENT_Y);
      for (Int y = 0; y < iPicHeight; y++)
      {
        for (Int x = 0; x < iPicWidth; x++)
        {
          Pel temp = ((pImgFrac[yFrac][xFrac][y*Stride + x] + IF_INTERNAL_OFFS) >> IF_FILTER_PREC);
          temp = Clip3<Pel>(0, 255, temp);
          lumaAddr[x] = temp;
        }
        lumaAddr += iStride;
      }

      pPicYuv->dump(pName, biDepth, true);
    }
  }

  pPicYuv->destroy();
  delete pPicYuv;
  pPicYuv = NULL;
#endif

}
#endif


#if OUTPUT_DCTIF_INTERPOLATED_FRAMES
Void TComPic::picDCTIF()
{
  Int Stride = getPicYuvRec()->getStride(COMPONENT_Y);
  Int Height = getPicYuvRec()->getHeight(COMPONENT_Y);
  Int Width  = getPicYuvRec()->getWidth(COMPONENT_Y);
  TComInterpolationFilter IF;
  Int filterSize = NTAPS_LUMA;
  Int halfFilterSize = (filterSize >> 1);
  const ChromaFormat chFmt = getPicYuvRec()->getChromaFormat();
  Int BitDepth = 8;
  BitDepths biDepths;
  biDepths.recon[0] = biDepths.recon[1] = 8;
  Pel *ptrSrc, *ptrDst, *ptrTmp[4];

  TComPicYuv* picYuvTmp[4];
  for (Int i = 0; i < 4; i++)
  {
    picYuvTmp[i] = new TComPicYuv;
    picYuvTmp[i]->create(Width, Height, chFmt, MAX_CU_SIZE, MAX_CU_SIZE, MAX_CU_DEPTH, true);
    ptrTmp[i] = picYuvTmp[i]->getAddr(COMPONENT_Y);
    picYuvTmp[i]->setBorderExtension(false);
    picYuvTmp[i]->extendPicBorder();
  }

  //initialization
  getPicYuvRec()->setBorderExtension(false);
  getPicYuvRec()->extendPicBorder();
  for (Int yFrac = 0; yFrac < 4; yFrac++)
  {
    for (Int xFrac = 0; xFrac < 4; xFrac++)
    {
      getPicYuvRec()->copyToPic(getPicYuvDCTIF(yFrac, xFrac));
      getPicYuvDCTIF(yFrac, xFrac)->setBorderExtension(false);
      getPicYuvDCTIF(yFrac, xFrac)->extendPicBorder();
    }
  }
  
  ptrSrc = getPicYuvRec()->getAddr(COMPONENT_Y);
  for (Int xFrac = 0; xFrac < 4; xFrac++)
  {
    IF.filterHor(COMPONENT_Y, ptrSrc, Stride, ptrTmp[xFrac], Stride, Width, Height, xFrac, false, chFmt, BitDepth);
  }

  std::string sPhase[] = { "0", "1", "2", "3" };
  for (Int yFrac = 0; yFrac < 4; yFrac++)
  {
    for (Int xFrac = 0; xFrac < 4; xFrac++)
    {
      if (yFrac == 0 && xFrac == 0)
      {
        continue;
      }
      ptrDst = getPicYuvDCTIF(yFrac, xFrac)->getAddr(COMPONENT_Y);
      IF.filterVer(COMPONENT_Y, ptrTmp[xFrac], Stride, ptrDst, Stride, Width, Height, yFrac, false, true, chFmt, BitDepth);
      std::string sName = "DCTIF_PH" + sPhase[yFrac] + sPhase[xFrac] + ".yuv";
      getPicYuvDCTIF(yFrac, xFrac)->dump(sName, biDepths, true);
    }
  }

  for (Int i = 0; i < 4; i++)
  {
    picYuvTmp[i]->destroy();
    delete picYuvTmp[i];
    picYuvTmp[i] = NULL;
  }

}
#endif

#if DRAW_PARTITION_MODE
Void TComPic::drawPartitionMode()
{
  TComPic* pcPic = this;

  pcPic->getPicYuvRec()->copyToPic(pcPic->getPicYuvRecTmp());
  for (Int iCtuAddr = 0; iCtuAddr < pcPic->getNumberOfCtusInFrame(); iCtuAddr++)
  {
    TComDataCU* pcCtu = pcPic->getCtu(iCtuAddr);
    Int idx = 0;
    while (idx < pcCtu->getTotalNumPart())
    {
      UInt uiDepth = pcCtu->getDepth(idx);
      Int iDepthIdx = 3 - uiDepth;
      Int iNumPart = pcCtu->getTotalNumPart() >> (2 * uiDepth);

      Int iNumInterPart = (pcCtu->getPartitionSize(idx) == SIZE_2Nx2N) ? 1 : 2;
      iNumInterPart = pcCtu->getPartitionSize(idx) == SIZE_NxN ? 4 : iNumInterPart;

      if (pcCtu->getPredictionMode(idx) == MODE_INTER)
      {
        Int width  = pcCtu->getWidth(idx);
        Int height = pcCtu->getHeight(idx);

        for (Int partIdx = 0; partIdx < iNumInterPart; partIdx++)
        {
          Int PUPartIdx = 0;
          Int width = pcCtu->getWidth(idx);
          Int height = pcCtu->getHeight(idx);
          if (pcCtu->getPartitionSize(idx) == SIZE_2Nx2N)
          {
            PUPartIdx = idx;
            width = pcCtu->getWidth(PUPartIdx);
            height = pcCtu->getHeight(PUPartIdx);
          }
          else if (pcCtu->getPartitionSize(idx) == SIZE_2NxN)
          {
            PUPartIdx = idx + partIdx * (iNumPart >> 1);
            width = pcCtu->getWidth(PUPartIdx);
            height = pcCtu->getHeight(PUPartIdx) >> 1;
          }
          else if (pcCtu->getPartitionSize(idx) == SIZE_Nx2N)
          {
            PUPartIdx = idx + partIdx * (iNumPart >> 2);
            width = pcCtu->getWidth(PUPartIdx) >> 1;
            height = pcCtu->getHeight(PUPartIdx);
          }
          else if (pcCtu->getPartitionSize(idx) == SIZE_2NxnU)
          {
            PUPartIdx = idx + partIdx * (iNumPart >> 3);
            width = pcCtu->getWidth(PUPartIdx);
            height = (partIdx == 0) ? pcCtu->getHeight(PUPartIdx) >> 2 : (pcCtu->getHeight(PUPartIdx) >> 2) + (pcCtu->getHeight(PUPartIdx) >> 1);
          }
          else if (pcCtu->getPartitionSize(idx) == SIZE_2NxnD)
          {
            PUPartIdx = idx + partIdx * ((iNumPart >> 3) + (iNumPart >> 1));
            width = pcCtu->getWidth(PUPartIdx);
            height = (partIdx == 1) ? pcCtu->getHeight(PUPartIdx) >> 2 : (pcCtu->getHeight(PUPartIdx) >> 2) + (pcCtu->getHeight(PUPartIdx) >> 1);;
          }
          else if (pcCtu->getPartitionSize(idx) == SIZE_nLx2N)
          {
            PUPartIdx = idx + partIdx * (iNumPart >> 4);
            width = (partIdx == 0) ? pcCtu->getWidth(PUPartIdx) >> 2 : (pcCtu->getWidth(PUPartIdx) >> 2) + (pcCtu->getWidth(PUPartIdx) >> 1);
            height = pcCtu->getHeight(PUPartIdx);
          }
          else if (pcCtu->getPartitionSize(idx) == SIZE_nRx2N)
          {
            PUPartIdx = idx + partIdx * ((iNumPart >> 4) + (iNumPart >> 2));
            width = (partIdx == 1) ? pcCtu->getWidth(PUPartIdx) >> 2 : (pcCtu->getWidth(PUPartIdx) >> 2) + (pcCtu->getWidth(PUPartIdx) >> 1);
            height = pcCtu->getHeight(PUPartIdx);
          }
          else if (pcCtu->getPartitionSize(idx) == SIZE_NxN)
          {
            PUPartIdx = idx + partIdx*(iNumPart >> 2);
            width = pcCtu->getWidth(PUPartIdx) >> 1;
            height = pcCtu->getHeight(PUPartIdx) >> 1;
          }
          Pel* LumaAddr = pcPic->getPicYuvRecTmp()->getAddr(COMPONENT_Y, iCtuAddr, PUPartIdx);
          Int  iStrideY = pcPic->getPicYuvRecTmp()->getStride(COMPONENT_Y);
          Pel* cbAddr = pcPic->getPicYuvRecTmp()->getAddr(COMPONENT_Cb, iCtuAddr, PUPartIdx);
          Pel* crAddr = pcPic->getPicYuvRecTmp()->getAddr(COMPONENT_Cr, iCtuAddr, PUPartIdx);
          Int  iStrideC = pcPic->getPicYuvRecTmp()->getStride(COMPONENT_Cb);
          if (pcCtu->getIFCNNFlag(PUPartIdx))
          {
            for (Int y = 0; y < height; y++)
            {
              for (Int x = 0; x < width; x++)
              {
                if (y == 0 || x == 0 || height - 1 - y == 0 || width - 1 - x == 0)
                {
                  LumaAddr[y*iStrideY + x] = 255;
                }
              }
            }
            for (Int y = 0; y < height / 2; y++)
            {
              for (Int x = 0; x < width / 2; x++)
              {
                if (y == 0 || x == 0 || height / 2 - 1 - y == 0 || width / 2 - 1 - x == 0)
                {
                  cbAddr[y*iStrideC + x] = 255;
                  crAddr[y*iStrideC + x] = 255;
                }
              }
            }
          }
          else
          {
            for (Int y = 0; y < height; y++)
            {
              for (Int x = 0; x < width; x++)
              {
                if (y == 0 || x == 0 || height - 1 - y == 0 || width - 1 - x == 0)
                {
                  LumaAddr[y*iStrideY + x] = 0;
                }
              }
            }
            for (Int y = 0; y < height / 2; y++)
            {
              for (Int x = 0; x < width / 2; x++)
              {
                if (y == 0 || x == 0 || height / 2 - 1 - y == 0 || width / 2 - 1 - x == 0)
                {
                  cbAddr[y*iStrideC + x] = 255;
                  crAddr[y*iStrideC + x] = 120;
                }
              }
            }
          }

          if (pcCtu->getbAllIntMvInCU(PUPartIdx))
          {
            for (Int y = 0; y < height; y++)
            {
              for (Int x = 0; x < width; x++)
              {
                if (y == 0 || x == 0 || height - 1 - y == 0 || width - 1 - x == 0)
                {
                  LumaAddr[y*iStrideY + x] = 255;
                }
              }
            }
            for (Int y = 0; y < height / 2; y++)
            {
              for (Int x = 0; x < width / 2; x++)
              {
                if (y == 0 || x == 0 || height / 2 - 1 - y == 0 || width / 2 - 1 - x == 0)
                {
                  cbAddr[y*iStrideC + x] = 0;
                  crAddr[y*iStrideC + x] = 255;
                }
              }
            }
          }
        }

      }
      else if (pcCtu->getPredictionMode(idx) == MODE_INTRA)
      {
        for (Int partIdx = 0; partIdx < iNumInterPart; partIdx++)
        {
          Int PUPartIdx = 0;
          Int width = pcCtu->getWidth(idx);
          Int height = pcCtu->getHeight(idx);
          if (pcCtu->getPartitionSize(idx) == SIZE_2Nx2N)
          {
            PUPartIdx = idx;
            width = pcCtu->getWidth(PUPartIdx);
            height = pcCtu->getHeight(PUPartIdx);
          }
          else if (pcCtu->getPartitionSize(idx) == SIZE_NxN)
          {
            PUPartIdx = idx + partIdx*(iNumPart >> 2);
            width = pcCtu->getWidth(PUPartIdx) >> 1;
            height = pcCtu->getHeight(PUPartIdx) >> 1;
          }
          Pel* LumaAddr = pcPic->getPicYuvRecTmp()->getAddr(COMPONENT_Y, iCtuAddr, PUPartIdx);
          Int  iStrideY = pcPic->getPicYuvRecTmp()->getStride(COMPONENT_Y);
          Pel* cbAddr = pcPic->getPicYuvRecTmp()->getAddr(COMPONENT_Cb, iCtuAddr, PUPartIdx);
          Pel* crAddr = pcPic->getPicYuvRecTmp()->getAddr(COMPONENT_Cr, iCtuAddr, PUPartIdx);
          Int  iStrideC = pcPic->getPicYuvRecTmp()->getStride(COMPONENT_Cb);
          for (Int y = 0; y < height; y++)
          {
            for (Int x = 0; x < width; x++)
            {
              if (y == 0 || x == 0 || height - 1 - y == 0 || width - 1 - x == 0)
              {
                LumaAddr[y*iStrideY + x] = 0;
              }
            }
          }
          for (Int y = 0; y < height / 2; y++)
          {
            for (Int x = 0; x < width / 2; x++)
            {
              if (y == 0 || x == 0 || height / 2 - 1 - y == 0 || width / 2 - 1 - x == 0)
              {
                cbAddr[y*iStrideC + x] = 0;
                crAddr[y*iStrideC + x] = 0;
              }
            }
          }
        }
      }
      idx += iNumPart;
    }
  }
}


Void TComPic::drawPartitionMode_Dec()
{
  TComPic* pcPic = this;

  pcPic->getPicYuvRec()->copyToPic(pcPic->getPicYuvRecTmp());
  for (Int iCtuAddr = 0; iCtuAddr < pcPic->getNumberOfCtusInFrame(); iCtuAddr++)
  {
    TComDataCU* pcCtu = pcPic->getCtu(iCtuAddr);
    Int idx = 0;
    while (idx < pcCtu->getTotalNumPart())
    {
      UInt uiDepth = pcCtu->getDepth(idx);
      Int iDepthIdx = 3 - uiDepth;
      Int iNumPart = pcCtu->getTotalNumPart() >> (2 * uiDepth);

      Int iNumInterPart = (pcCtu->getPartitionSize(idx) == SIZE_2Nx2N) ? 1 : 2;
      iNumInterPart = pcCtu->getPartitionSize(idx) == SIZE_NxN ? 4 : iNumInterPart;

      Pel* LumaAddr = pcPic->getPicYuvRecTmp()->getAddr(COMPONENT_Y, iCtuAddr, idx);
      Int  iStrideY = pcPic->getPicYuvRecTmp()->getStride(COMPONENT_Y);
      Pel* cbAddr = pcPic->getPicYuvRecTmp()->getAddr(COMPONENT_Cb, iCtuAddr, idx);
      Pel* crAddr = pcPic->getPicYuvRecTmp()->getAddr(COMPONENT_Cr, iCtuAddr, idx);
      Int  iStrideC = pcPic->getPicYuvRecTmp()->getStride(COMPONENT_Cb);

      Bool bAllIntMvInCU = true;
      for (Int partIdx = 0; partIdx < iNumInterPart; partIdx++)
      {
        Int PUPartIdx = 0;
        if (pcCtu->getPartitionSize(idx) == SIZE_2Nx2N)
        {
          PUPartIdx = idx;
        }
        else if (pcCtu->getPartitionSize(idx) == SIZE_2NxN)
        {
          PUPartIdx = idx + partIdx * (iNumPart >> 1);
        }
        else if (pcCtu->getPartitionSize(idx) == SIZE_Nx2N)
        {
          PUPartIdx = idx + partIdx * (iNumPart >> 2);
        }
        else if (pcCtu->getPartitionSize(idx) == SIZE_2NxnU)
        {
          PUPartIdx = idx + partIdx * (iNumPart >> 3);
        }
        else if (pcCtu->getPartitionSize(idx) == SIZE_2NxnD)
        {
          PUPartIdx = idx + partIdx * ((iNumPart >> 3) + (iNumPart >> 1));
        }
        else if (pcCtu->getPartitionSize(idx) == SIZE_nLx2N)
        {
          PUPartIdx = idx + partIdx * (iNumPart >> 4);
        }
        else if (pcCtu->getPartitionSize(idx) == SIZE_nRx2N)
        {
          PUPartIdx = idx + partIdx * ((iNumPart >> 4) + (iNumPart >> 2));
        }
        else if (pcCtu->getPartitionSize(idx) == SIZE_NxN)
        {
          PUPartIdx = idx + partIdx*(iNumPart >> 2);
        }
        bAllIntMvInCU &= pcCtu->getbAllIntMvInCU(PUPartIdx);
      }

      if (pcCtu->getPredictionMode(idx) == MODE_INTER && !bAllIntMvInCU)
      {
        Int width = pcCtu->getWidth(idx);
        Int height = pcCtu->getHeight(idx);

        if (pcCtu->getIFCNNFlag(idx))
        {
          for (Int y = 0; y < height; y++)
          {
            for (Int x = 0; x < width; x++)
            {
              if (y == 0 || x == 0 || height - 1 - y == 0 || width - 1 - x == 0)
              {
                LumaAddr[y*iStrideY + x] = 255;
              }
            }
          }
          for (Int y = 0; y < height / 2; y++)
          {
            for (Int x = 0; x < width / 2; x++)
            {
              if (y == 0 || x == 0 || height / 2 - 1 - y == 0 || width / 2 - 1 - x == 0)
              {
                cbAddr[y*iStrideC + x] = 255;
                crAddr[y*iStrideC + x] = 255;
              }
            }
          }
        }
        else
        {
          for (Int y = 0; y < height; y++)
          {
            for (Int x = 0; x < width; x++)
            {
              if (y == 0 || x == 0 || height - 1 - y == 0 || width - 1 - x == 0)
              {
                LumaAddr[y*iStrideY + x] = 0;
              }
            }
          }
          for (Int y = 0; y < height / 2; y++)
          {
            for (Int x = 0; x < width / 2; x++)
            {
              if (y == 0 || x == 0 || height / 2 - 1 - y == 0 || width / 2 - 1 - x == 0)
              {
                cbAddr[y*iStrideC + x] = 255;
                crAddr[y*iStrideC + x] = 120;
              }
            }
          }
        }
      }

      idx += iNumPart;
    }
  }
}
#endif

//! \}
