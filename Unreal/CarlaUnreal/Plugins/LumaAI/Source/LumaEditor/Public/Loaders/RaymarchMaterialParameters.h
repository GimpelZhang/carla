// # Copyright 2023 Luma AI
#pragma once

#include "CoreMinimal.h"

namespace LumaConstants
{
const static FName T_O = "LMC_1";
const static FName T_O_L = "LMC_2";
const static FName T_F_1 = "LMC_3";
const static FName T_F_2 = "LMC_4";
const static FName MinStepSize = "LMC_5";
const static FName LindispStep = "LMC_6";
const static FName MaxSteps = "LMC_7";
const static FName EnableInfSample = "LMC_8";
const static FName MaterialFunctionColor = "LMC_9";
const static FName MaterialFunctionAlpha = "LMC_10";
const static FName MaterialFunctionNormal = "LMC_11";
const static FName MaterialFunctionWPO = "LMC_12";
const static FName MaterialFunctionPDO = "LMC_13";
const static FName EntryPos = "LMC_14";
const static FName Thickness = "LMC_15";
const static FName PixelDepth = "LMC_16";
const static FName CropMin = "CropMin";
const static FName CropMax = "CropMax";
const static FName CropCenter = "CropCenter";
const static FName CropRotation = "CropRotation";
const static FName QualityMode = "QualityMode";
const static FName Exposure = "Exposure";

const static FName GSNumGaussians = "NumGaussians";
const static FName GSMaterial = "Material";
const static FName GSColorTex = "LMG_1";
const static FName GSPosTex = "LMG_2";
const static FName GSRotTex = "LMG_3";
const static FName GSScaleTex = "LMG_4";
const static FName GSChunkOffset = "LMG_5";
const static FName GSTextureSize = "LMG_6";
const static FName GSInvTextureSize = "LMG_7";
const static FName GSCropActor = "CropBox";
const static FName GSCropActorEnabled = "CropBoxEnabled";
const static FName GSCullActor1 = "CullBox1";
const static FName GSCullActor2 = "CullBox2";
const static FName GSCullActor3 = "CullBox3";
const static FName GSCullActor4 = "CullBox4";
const static FName GSCullActorEnabled1 = "CullBox1Enabled";
const static FName GSCullActorEnabled2 = "CullBox2Enabled";
const static FName GSCullActorEnabled3 = "CullBox3Enabled";
const static FName GSCullActorEnabled4 = "CullBox4Enabled";
const static FName GSSplatScale = "SplatScale";
constexpr static int MaxNumGaussiansSingleChunk = 1900000;

constexpr static float MinStepSizeDefault = 0.005f;
constexpr static float LindispStepDefault = 0.0012f;
constexpr static int MaxStepsDefault = 64;
constexpr static int EnableInfSampleDefault = 1;
constexpr static int LumaShaderVersion = 1;


} // namespace LumaConstants
