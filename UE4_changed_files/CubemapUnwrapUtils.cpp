// Copyright Epic Games, Inc. All Rights Reserved.

/*=============================================================================
	CubemapUnwapUtils.cpp: Pixel and Vertex shader to render a cube map as 2D texture
=============================================================================*/

#include "CubemapUnwrapUtils.h"
#include "CanvasItem.h"
#include "CanvasTypes.h"
#include "ShaderParameterUtils.h"
#include "SimpleElementShaders.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/TextureCube.h"
#include "Engine/TextureRenderTargetCube.h"
#include "PipelineStateCache.h"

IMPLEMENT_SHADER_TYPE(,FCubemapTexturePropertiesVS,TEXT("/Engine/Private/SimpleElementVertexShader.usf"),TEXT("Main"),SF_Vertex);
IMPLEMENT_SHADER_TYPE(template<>,FCubemapTexturePropertiesPS<false>,TEXT("/Engine/Private/SimpleElementPixelShader.usf"),TEXT("CubemapTextureProperties"),SF_Pixel);
IMPLEMENT_SHADER_TYPE(template<>,FCubemapTexturePropertiesPS<true>,TEXT("/Engine/Private/SimpleElementPixelShader.usf"),TEXT("CubemapTextureProperties"),SF_Pixel);
IMPLEMENT_SHADER_TYPE(,FIESLightProfilePS,TEXT("/Engine/Private/SimpleElementPixelShader.usf"),TEXT("IESLightProfileMain"),SF_Pixel);

namespace CubemapHelpers
{
	/**
	* Helper function to create an unwrapped 2D image of the cube map ( longitude/latitude )
	* This version takes explicitly passed properties of the source object, as the sources have different APIs.
	* @param	TextureResource		Source FTextureResource object.
	* @param	AxisDimenion		axis length of the cube.
	* @param	SourcePixelFormat	pixel format of the source.
	* @param	BitsOUT				Raw bits of the 2D image bitmap.
	* @param	SizeXOUT			Filled with the X dimension of the output bitmap.
	* @param	SizeYOUT			Filled with the Y dimension of the output bitmap.
	* @return						true on success.
	* @param	FormatOUT			Filled with the pixel format of the output bitmap.
	*/
	bool GenerateLongLatUnwrap(const FTextureResource* TextureResource, const uint32 AxisDimenion, const EPixelFormat SourcePixelFormat, TArray64<uint8>& BitsOUT, FIntPoint& SizeOUT, EPixelFormat& FormatOUT)
	{
		TRefCountPtr<FBatchedElementParameters> BatchedElementParameters;
		BatchedElementParameters = new FMipLevelBatchedElementParameters((float)0, true);
		const FIntPoint LongLatDimensions(AxisDimenion * 2, AxisDimenion);

		// If the source format is 8 bit per channel or less then select a LDR target format.
		const EPixelFormat TargetPixelFormat = CalculateImageBytes(1, 1, 0, SourcePixelFormat) <= 4 ? PF_B8G8R8A8 : PF_FloatRGBA;

		UTextureRenderTarget2D* RenderTargetLongLat = NewObject<UTextureRenderTarget2D>();
		check(RenderTargetLongLat);
		RenderTargetLongLat->AddToRoot();
		RenderTargetLongLat->ClearColor = FLinearColor(0.0f, 0.0f, 0.0f, 0.0f);
		RenderTargetLongLat->InitCustomFormat(LongLatDimensions.X, LongLatDimensions.Y, TargetPixelFormat, false);
		RenderTargetLongLat->TargetGamma = 0;
		FRenderTarget* RenderTarget = RenderTargetLongLat->GameThread_GetRenderTargetResource();

		FCanvas* Canvas = new FCanvas(RenderTarget, NULL, 0, 0, 0, GMaxRHIFeatureLevel);
		Canvas->SetRenderTarget_GameThread(RenderTarget);

		// Clear the render target to black
		Canvas->Clear(FLinearColor(0, 0, 0, 0));

		FCanvasTileItem TileItem(FVector2D(0.0f, 0.0f), TextureResource, FVector2D(LongLatDimensions.X, LongLatDimensions.Y), FLinearColor::White);
		TileItem.BatchedElementParameters = BatchedElementParameters;
		TileItem.BlendMode = SE_BLEND_Opaque;
		Canvas->DrawItem(TileItem);

		Canvas->Flush_GameThread();
		FlushRenderingCommands();
		Canvas->SetRenderTarget_GameThread(NULL);
		FlushRenderingCommands();
		
		int32 ImageBytes = CalculateImageBytes(LongLatDimensions.X, LongLatDimensions.Y, 0, TargetPixelFormat);

		BitsOUT.AddUninitialized(ImageBytes);

		bool bReadSuccess = false;
		switch (TargetPixelFormat)
		{
			case PF_B8G8R8A8:
				bReadSuccess = RenderTarget->ReadPixelsPtr((FColor*)BitsOUT.GetData());
			break;
			case PF_FloatRGBA:
				{
					TArray<FFloat16Color> FloatColors;
					bReadSuccess = RenderTarget->ReadFloat16Pixels(FloatColors);
					FMemory::Memcpy(BitsOUT.GetData(), FloatColors.GetData(), ImageBytes);
				}
			break;
		}
		// Clean up.
		RenderTargetLongLat->ReleaseResource();
		RenderTargetLongLat->RemoveFromRoot();
		RenderTargetLongLat = NULL;
		delete Canvas;

		SizeOUT = LongLatDimensions;
		FormatOUT = TargetPixelFormat;
		if (bReadSuccess == false)
		{
			// Reading has failed clear output buffer.
			BitsOUT.Empty();
		}

		return bReadSuccess;
	}

	bool GenerateLongLatUnwrap(const UTextureCube* CubeTexture, TArray64<uint8>& BitsOUT, FIntPoint& SizeOUT, EPixelFormat& FormatOUT)
	{
		check(CubeTexture != NULL);
		return GenerateLongLatUnwrap(CubeTexture->Resource, CubeTexture->GetSizeX(), CubeTexture->GetPixelFormat(), BitsOUT, SizeOUT, FormatOUT);
	}

	bool GenerateLongLatUnwrap(const UTextureRenderTargetCube* CubeTarget, TArray64<uint8>& BitsOUT, FIntPoint& SizeOUT, EPixelFormat& FormatOUT)
	{
		check(CubeTarget != NULL);
		return GenerateLongLatUnwrap(CubeTarget->Resource, CubeTarget->SizeX, CubeTarget->GetFormat(), BitsOUT, SizeOUT, FormatOUT);
	}
}

void FCubemapTexturePropertiesVS::SetParameters( FRHICommandList& RHICmdList, const FMatrix& TransformValue )
{
	SetShaderValue(RHICmdList, RHICmdList.GetBoundVertexShader(), Transform, TransformValue);
}

template<bool bHDROutput>
void FCubemapTexturePropertiesPS<bHDROutput>::SetParameters( FRHICommandList& RHICmdList, const FTexture* Texture, const FMatrix& ColorWeightsValue, float MipLevel, float GammaValue )
{
	SetTextureParameter(RHICmdList, RHICmdList.GetBoundPixelShader(),CubeTexture,CubeTextureSampler,Texture);

	FVector4 PackedProperties0Value(MipLevel, 0, 0, 0);
	SetShaderValue(RHICmdList, RHICmdList.GetBoundPixelShader(), PackedProperties0, PackedProperties0Value);
	SetShaderValue(RHICmdList, RHICmdList.GetBoundPixelShader(), ColorWeights, ColorWeightsValue);
	SetShaderValue(RHICmdList, RHICmdList.GetBoundPixelShader(), Gamma, GammaValue);
}

void FMipLevelBatchedElementParameters::BindShaders(FRHICommandList& RHICmdList, FGraphicsPipelineStateInitializer& GraphicsPSOInit, ERHIFeatureLevel::Type InFeatureLevel, const FMatrix& InTransform, const float InGamma, const FMatrix& ColorWeights, const FTexture* Texture)
{
	if(bHDROutput)
	{
		BindShaders<FCubemapTexturePropertiesPS<true> >(RHICmdList, GraphicsPSOInit, InFeatureLevel, InTransform, InGamma, ColorWeights, Texture);
	}
	else
	{
		BindShaders<FCubemapTexturePropertiesPS<false> >(RHICmdList, GraphicsPSOInit, InFeatureLevel, InTransform, InGamma, ColorWeights, Texture);
	}
}

template<typename TPixelShader>
void FMipLevelBatchedElementParameters::BindShaders(FRHICommandList& RHICmdList, FGraphicsPipelineStateInitializer& GraphicsPSOInit, ERHIFeatureLevel::Type InFeatureLevel, const FMatrix& InTransform, const float InGamma, const FMatrix& ColorWeights, const FTexture* Texture)
{
	GraphicsPSOInit.BlendState = TStaticBlendState<>::GetRHI();

	TShaderMapRef<FCubemapTexturePropertiesVS> VertexShader(GetGlobalShaderMap(InFeatureLevel));
	TShaderMapRef<TPixelShader> PixelShader(GetGlobalShaderMap(InFeatureLevel));
	
	GraphicsPSOInit.BoundShaderState.VertexDeclarationRHI = GSimpleElementVertexDeclaration.VertexDeclarationRHI;
	GraphicsPSOInit.BoundShaderState.VertexShaderRHI = VertexShader.GetVertexShader();
	GraphicsPSOInit.BoundShaderState.PixelShaderRHI = PixelShader.GetPixelShader();
	GraphicsPSOInit.PrimitiveType = PT_TriangleList;

	SetGraphicsPipelineState(RHICmdList, GraphicsPSOInit, EApplyRendertargetOption::ForceApply);

	VertexShader->SetParameters(RHICmdList, InTransform);
	PixelShader->SetParameters(RHICmdList, Texture, ColorWeights, MipLevel, InGamma);
}

void FIESLightProfilePS::SetParameters( FRHICommandList& RHICmdList, const FTexture* Texture, float InBrightnessInLumens )
{
	FRHIPixelShader* ShaderRHI = RHICmdList.GetBoundPixelShader();
	SetTextureParameter(RHICmdList, ShaderRHI, IESTexture, IESTextureSampler, Texture);

	SetShaderValue(RHICmdList, ShaderRHI, BrightnessInLumens, InBrightnessInLumens);
}

void FIESLightProfileBatchedElementParameters::BindShaders( FRHICommandList& RHICmdList, FGraphicsPipelineStateInitializer& GraphicsPSOInit, ERHIFeatureLevel::Type InFeatureLevel, const FMatrix& InTransform, const float InGamma, const FMatrix& ColorWeights, const FTexture* Texture )
{
	GraphicsPSOInit.BlendState = TStaticBlendState<>::GetRHI();

	TShaderMapRef<FSimpleElementVS> VertexShader(GetGlobalShaderMap(InFeatureLevel));
	TShaderMapRef<FIESLightProfilePS> PixelShader(GetGlobalShaderMap(InFeatureLevel));

	GraphicsPSOInit.BoundShaderState.VertexDeclarationRHI = GSimpleElementVertexDeclaration.VertexDeclarationRHI;
	GraphicsPSOInit.BoundShaderState.VertexShaderRHI = VertexShader.GetVertexShader();
	GraphicsPSOInit.BoundShaderState.PixelShaderRHI = PixelShader.GetPixelShader();
	GraphicsPSOInit.PrimitiveType = PT_TriangleList;

	SetGraphicsPipelineState(RHICmdList, GraphicsPSOInit, EApplyRendertargetOption::ForceApply);

	VertexShader->SetParameters(RHICmdList, InTransform);
	PixelShader->SetParameters(RHICmdList, Texture, BrightnessInLumens);
}

////////////////////////////

IMPLEMENT_SHADER_TYPE(,FCubemapTexturePropertiesVSFisheye,TEXT("/Engine/Private/SimpleElementVertexShader.usf"),TEXT("Main"),SF_Vertex);
IMPLEMENT_SHADER_TYPE(template<>,FCubemapTexturePropertiesPSFisheye<false>,TEXT("/Engine/Private/SimpleElementPixelShader.usf"),TEXT("CubemapTexturePropertiesFisheye"),SF_Pixel);
IMPLEMENT_SHADER_TYPE(template<>,FCubemapTexturePropertiesPSFisheye<true>,TEXT("/Engine/Private/SimpleElementPixelShader.usf"),TEXT("CubemapTexturePropertiesFisheye"),SF_Pixel);

namespace CubemapHelpersFisheye
{
    /**
    * Helper function to create an unwrapped 2D image of the cube map ( longitude/latitude )
    * This version takes explicitly passed properties of the source object, as the sources have different APIs.
    * @param	TextureResource		Source FTextureResource object.
    * @param	SourcePixelFormat	pixel format of the source.
    * @param	BitsOUT				Raw bits of the 2D image bitmap.
    * @param	SizeXOUT			Filled with the X dimension of the output bitmap.
    * @param	SizeYOUT			Filled with the Y dimension of the output bitmap.
    * @return						true on success.
    * @param	FormatOUT			Filled with the pixel format of the output bitmap.
    */
    bool GenerateLongLatUnwrapFisheye(const FTextureResource* TextureResource, const EPixelFormat SourcePixelFormat, TArray64<uint8>& BitsOUT, FFisheyeParams &FisheyeParams, FIntPoint& SizeOUT, EPixelFormat& FormatOUT)
    {
        TRefCountPtr<FBatchedElementParameters> BatchedElementParameters;
        BatchedElementParameters = new FMipLevelBatchedElementParametersFisheye(
                    (float)0,
                    FisheyeParams.DistortionCoeffs,
                    FisheyeParams.CameraMatrix,
                    FisheyeParams.ImageSize,
                    FisheyeParams.MaxAngle,
                    true);

        // If the source format is 8 bit per channel or less then select a LDR target format.
        const EPixelFormat TargetPixelFormat = CalculateImageBytes(1, 1, 0, SourcePixelFormat) <= 4 ? PF_B8G8R8A8 : PF_FloatRGBA;

        UTextureRenderTarget2D* RenderTargetLongLat = NewObject<UTextureRenderTarget2D>();
        check(RenderTargetLongLat);
        RenderTargetLongLat->AddToRoot();
        RenderTargetLongLat->ClearColor = FLinearColor(0.0f, 0.0f, 0.0f, 0.0f);
        RenderTargetLongLat->InitCustomFormat(FisheyeParams.ImageSize.X, FisheyeParams.ImageSize.Y, TargetPixelFormat, false);
        RenderTargetLongLat->TargetGamma = 0;
        FRenderTarget* RenderTarget = RenderTargetLongLat->GameThread_GetRenderTargetResource();

        FCanvas* Canvas = new FCanvas(RenderTarget, NULL, 0, 0, 0, GMaxRHIFeatureLevel);
        Canvas->SetRenderTarget_GameThread(RenderTarget);

        // Clear the render target to black
        Canvas->Clear(FLinearColor(0, 0, 0, 0));

        FCanvasTileItem TileItem(FVector2D(0.0f, 0.0f), TextureResource, FVector2D(FisheyeParams.ImageSize.X, FisheyeParams.ImageSize.Y), FLinearColor::White);
        TileItem.BatchedElementParameters = BatchedElementParameters;
        TileItem.BlendMode = SE_BLEND_Opaque;
        Canvas->DrawItem(TileItem);

        Canvas->Flush_GameThread();
        FlushRenderingCommands();
        Canvas->SetRenderTarget_GameThread(NULL);
        FlushRenderingCommands();

        int32 ImageBytes = CalculateImageBytes(FisheyeParams.ImageSize.X, FisheyeParams.ImageSize.Y, 0, TargetPixelFormat);

        BitsOUT.AddUninitialized(ImageBytes);

        bool bReadSuccess = false;
        switch (TargetPixelFormat)
        {
            case PF_B8G8R8A8:
                bReadSuccess = RenderTarget->ReadPixelsPtr((FColor*)BitsOUT.GetData());
            break;
            case PF_FloatRGBA:
                {
                    TArray<FFloat16Color> FloatColors;
                    bReadSuccess = RenderTarget->ReadFloat16Pixels(FloatColors);
                    FMemory::Memcpy(BitsOUT.GetData(), FloatColors.GetData(), ImageBytes);
                }
            break;
        }
        // Clean up.
        RenderTargetLongLat->ReleaseResource();
        RenderTargetLongLat->RemoveFromRoot();
        RenderTargetLongLat = NULL;
        delete Canvas;

        SizeOUT = FisheyeParams.ImageSize;
        FormatOUT = TargetPixelFormat;
        if (bReadSuccess == false)
        {
            // Reading has failed clear output buffer.
            BitsOUT.Empty();
        }

        return bReadSuccess;
    }

    bool GenerateLongLatUnwrapFisheye(const UTextureCube* CubeTexture, TArray64<uint8>& BitsOUT, FFisheyeParams &FisheyeParams, FIntPoint& SizeOUT, EPixelFormat& FormatOUT)
    {
        check(CubeTexture != NULL);
        return GenerateLongLatUnwrapFisheye(CubeTexture->Resource, CubeTexture->GetPixelFormat(), BitsOUT, FisheyeParams, SizeOUT, FormatOUT);
    }

    bool GenerateLongLatUnwrapFisheye(const UTextureRenderTargetCube* CubeTarget, TArray64<uint8>& BitsOUT, FFisheyeParams &FisheyeParams, FIntPoint& SizeOUT, EPixelFormat& FormatOUT)
    {
        check(CubeTarget != NULL);
        return GenerateLongLatUnwrapFisheye(CubeTarget->Resource, CubeTarget->GetFormat(), BitsOUT, FisheyeParams, SizeOUT, FormatOUT);
    }
}

void FCubemapTexturePropertiesVSFisheye::SetParameters( FRHICommandList& RHICmdList, const FMatrix& TransformValue )
{
    SetShaderValue(RHICmdList, RHICmdList.GetBoundVertexShader(), Transform, TransformValue);
}

template<bool bHDROutput>
void FCubemapTexturePropertiesPSFisheye<bHDROutput>::SetParameters(
        FRHICommandList& RHICmdList,
        const FTexture* Texture,
        const FMatrix& ColorWeightsValue,
        float MipLevel,
        float GammaValue,
        const FVector4 &DistortionCoeffsValue,
        const FVector4 &CameraMatrixValue,
        const FIntPoint &ImageSizeValue,
        float MaxAngleValue)
{
    SetTextureParameter(RHICmdList, RHICmdList.GetBoundPixelShader(),CubeTexture,CubeTextureSampler,Texture);

    FVector4 PackedProperties0Value(MipLevel, 0, 0, 0);
    SetShaderValue(RHICmdList, RHICmdList.GetBoundPixelShader(), PackedProperties0, PackedProperties0Value);
    SetShaderValue(RHICmdList, RHICmdList.GetBoundPixelShader(), ColorWeights, ColorWeightsValue);
    SetShaderValue(RHICmdList, RHICmdList.GetBoundPixelShader(), Gamma, GammaValue);

    SetShaderValue(RHICmdList, RHICmdList.GetBoundPixelShader(), DistortionCoeffs, DistortionCoeffsValue);
    SetShaderValue(RHICmdList, RHICmdList.GetBoundPixelShader(), CameraMatrix, CameraMatrixValue);

    FVector2D ImageSizeF(ImageSizeValue.X, ImageSizeValue.Y);
    SetShaderValue(RHICmdList, RHICmdList.GetBoundPixelShader(), ImageSize, ImageSizeF);
    SetShaderValue(RHICmdList, RHICmdList.GetBoundPixelShader(), MaxAngle, MaxAngleValue);
}

void FMipLevelBatchedElementParametersFisheye::BindShaders(FRHICommandList& RHICmdList, FGraphicsPipelineStateInitializer& GraphicsPSOInit, ERHIFeatureLevel::Type InFeatureLevel, const FMatrix& InTransform, const float InGamma, const FMatrix& ColorWeights, const FTexture* Texture)
{
    if(bHDROutput)
    {
        BindShaders<FCubemapTexturePropertiesPSFisheye<true> >(RHICmdList, GraphicsPSOInit, InFeatureLevel, InTransform, InGamma, ColorWeights, Texture);
    }
    else
    {
        BindShaders<FCubemapTexturePropertiesPSFisheye<false> >(RHICmdList, GraphicsPSOInit, InFeatureLevel, InTransform, InGamma, ColorWeights, Texture);
    }
}

template<typename TPixelShader>
void FMipLevelBatchedElementParametersFisheye::BindShaders(FRHICommandList& RHICmdList, FGraphicsPipelineStateInitializer& GraphicsPSOInit, ERHIFeatureLevel::Type InFeatureLevel, const FMatrix& InTransform, const float InGamma, const FMatrix& ColorWeights, const FTexture* Texture)
{
    GraphicsPSOInit.BlendState = TStaticBlendState<>::GetRHI();

    TShaderMapRef<FCubemapTexturePropertiesVSFisheye> VertexShader(GetGlobalShaderMap(InFeatureLevel));
    TShaderMapRef<TPixelShader> PixelShader(GetGlobalShaderMap(InFeatureLevel));

    GraphicsPSOInit.BoundShaderState.VertexDeclarationRHI = GSimpleElementVertexDeclaration.VertexDeclarationRHI;
    GraphicsPSOInit.BoundShaderState.VertexShaderRHI = VertexShader.GetVertexShader();
    GraphicsPSOInit.BoundShaderState.PixelShaderRHI = PixelShader.GetPixelShader();
    GraphicsPSOInit.PrimitiveType = PT_TriangleList;

    SetGraphicsPipelineState(RHICmdList, GraphicsPSOInit, EApplyRendertargetOption::ForceApply);

    VertexShader->SetParameters(RHICmdList, InTransform);
    PixelShader->SetParameters(RHICmdList,
                               Texture,
                               ColorWeights,
                               MipLevel,
                               InGamma,
                               DistortionCoeffs,
                               CameraMatrix,
                               ImageSize,
                               MaxAngle);
}