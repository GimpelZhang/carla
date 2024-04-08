// # Copyright 2023 Luma AI

#include "Loaders/LumaLoader.h"
#include "Materials/MaterialExpressionTextureSample.h"
#include "Materials/MaterialExpressionConstant3Vector.h"
#include "Materials/MaterialExpressionConstant.h"
#ifdef WITH_EDITOR
#include "ComponentReregisterContext.h"
#include "Factories/MaterialFactoryNew.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Kismet2/CompilerResultsLog.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Materials/MaterialExpressionAdd.h"
#include "Materials/MaterialExpressionCameraPositionWS.h"
#include "Materials/MaterialExpressionComponentMask.h"
#include "Materials/MaterialExpressionCustom.h"
#include "Materials/MaterialExpressionDistance.h"
#include "Materials/MaterialExpressionDivide.h"
#include "Materials/MaterialExpressionDotProduct.h"
#include "Materials/MaterialExpressionIf.h"
#include "Materials/MaterialExpressionLinearInterpolate.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionNormalize.h"
#include "Materials/MaterialExpressionPixelDepth.h"
#include "Materials/MaterialExpressionSaturate.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "Materials/MaterialExpressionSubtract.h"
#include "Materials/MaterialExpressionTextureObject.h"
#include "Materials/MaterialExpressionTransform.h"
#include "Materials/MaterialExpressionTransformPosition.h"
#include "Materials/MaterialExpressionVectorParameter.h"
#include "Materials/MaterialExpressionWorldPosition.h"
#endif
#include "ObjectTools.h"
#include "sstream"
#include "string"
#include "iostream"
#include "LumaField.h"
#include "Factories/MaterialFunctionFactoryNew.h"
#include "Materials/MaterialExpressionFunctionOutput.h"
#include "Loaders/RaymarchMaterialParameters.h"
#include "Engine/VolumeTexture.h"
#include "Materials/MaterialExpressionFunctionInput.h"
#include "LumaDLLLoader.h"
#include "LumaRenderSDKPublic.h"


ULumaLoader* ULumaLoader::Get()
{
	// Maybe get a singleton going on here?
	return NewObject<ULumaLoader>();
}


struct TextureContainer
{
	UVolumeTexture* T_O = nullptr;
	UVolumeTexture* T_O_L = nullptr;
	UVolumeTexture* T_S_O = nullptr;
	UVolumeTexture* T_S_O_L = nullptr;
	UVolumeTexture* T_F_1 = nullptr;
	UTexture2D* T_F_2 = nullptr;
};

UMaterialFunction* CreateMaterialFunctionAssetPackage(FString AssetName, FString PackageRoot)
{
	// Create an unreal material asset
	auto MaterialFactory = NewObject<UMaterialFunctionFactoryNew>();

	FString PackageName = PackageRoot + AssetName;
	UPackage* AssetPackage = CreatePackage( *PackageName);
	
	auto UnrealObject = static_cast<UMaterialFunction*>(MaterialFactory->FactoryCreateNew(UMaterialFunction::StaticClass(),
		AssetPackage,
		*AssetName, RF_Standalone | RF_Public, NULL, GWarn));

	FAssetRegistryModule::AssetCreated(UnrealObject);
	AssetPackage->FullyLoad();
	AssetPackage->SetDirtyFlag(true);

	return UnrealObject;
}

UMaterial* CreateMaterialAssetPackage(FString AssetName, FString PackageRoot)
{
	// Create an unreal material asset
	auto MaterialFactory = NewObject<UMaterialFactoryNew>();

	FString PackageName = PackageRoot + AssetName;
	UPackage* AssetPackage = CreatePackage( *PackageName);
	
	auto UnrealObject = static_cast<UMaterial*>(MaterialFactory->FactoryCreateNew(UMaterial::StaticClass(),
		AssetPackage,
		*AssetName, RF_Standalone | RF_Public, NULL, GWarn));

	FAssetRegistryModule::AssetCreated(UnrealObject);
	AssetPackage->FullyLoad();
	AssetPackage->SetDirtyFlag(true);

	return UnrealObject;
}

UVolumeTexture* CreateUVolumeTextureAssetPackage(FString AssetName, FString PackageRoot)
{
	// Create an unreal material asset
	auto MaterialFactory = NewObject<UMaterialFunctionFactoryNew>();

	FString PackageName = PackageRoot + AssetName;
	UPackage* AssetPackage = CreatePackage( *PackageName);
	
	auto UnrealObject = NewObject<UVolumeTexture>(AssetPackage,
				*AssetName,
				RF_Public | RF_Standalone);

	FAssetRegistryModule::AssetCreated(UnrealObject);
	AssetPackage->FullyLoad();
	AssetPackage->SetDirtyFlag(true);

	return UnrealObject;
}

UTexture2D* CreateUTexture2DAssetPackage(FString AssetName, FString PackageRoot)
{
	// Create an unreal material asset
	auto MaterialFactory = NewObject<UMaterialFunctionFactoryNew>();

	FString PackageName = PackageRoot + AssetName;
	UPackage* AssetPackage = CreatePackage( *PackageName);
	
	auto UnrealObject = NewObject<UTexture2D>(AssetPackage,
				*AssetName,
				RF_Public | RF_Standalone);

	FAssetRegistryModule::AssetCreated(UnrealObject);
	AssetPackage->FullyLoad();
	AssetPackage->SetDirtyFlag(true);

	return UnrealObject;
}


UBlueprint* CreateLumaFieldAssetPackage(FString AssetName, FString PackageRoot, UPackage** BlueprintPackage)
{
	FString PackageName = PackageRoot + AssetName;
	*BlueprintPackage = CreatePackage( *PackageName);
	UBlueprint* UnrealObject = FKismetEditorUtilities::CreateBlueprint(ALumaField::StaticClass(), *BlueprintPackage, *AssetName,
		BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());

	return UnrealObject;
}


void Setup2DTexture(UTexture2D*& OutTexture, EPixelFormat PixelFormat, FIntPoint Dimensions, uint8* InSourceArray)
{
	if(InSourceArray == nullptr)
		return;

	if (Dimensions.X == 0 && Dimensions.Y == 0)
		return;
	
	// Newly created Volume textures have this null'd
	if (!OutTexture->GetPlatformData())
	{
		OutTexture->SetPlatformData(new FTexturePlatformData());
	}
	// Set Dimensions and Pixel format.
	OutTexture->GetPlatformData()->SizeX = Dimensions.X;
	OutTexture->GetPlatformData()->SizeY = Dimensions.Y;
	OutTexture->GetPlatformData()->SetNumSlices(1);
	OutTexture->GetPlatformData()->PixelFormat = PixelFormat;
	OutTexture->CompressionSettings = TC_HDR;
	OutTexture->SRGB = false;
	OutTexture->NeverStream = true;

	// Create texture
	int BlockBytes = GPixelFormats[PixelFormat].BlockBytes;
	int TotalBytes = Dimensions.X * Dimensions.Y * BlockBytes;
	OutTexture->GetPlatformData()->Mips.Add(new FTexture2DMipMap());
	auto Mip = OutTexture->GetPlatformData()->Mips.Last();
	Mip.SizeX = Dimensions.X;
	Mip.SizeY = Dimensions.Y;

	Mip.BulkData.Lock(LOCK_READ_WRITE);

	uint8* TextureData = Mip.BulkData.Realloc(TotalBytes);

	FMemory::Memcpy(TextureData, InSourceArray, TotalBytes);

	Mip.BulkData.Unlock();

	OutTexture->Source.Init(Dimensions.X, Dimensions.Y, 1, 1, TSF_RGBA16F, InSourceArray);

	OutTexture->UpdateResource();
	OutTexture->MarkPackageDirty();
	FAssetRegistryModule::AssetCreated(OutTexture);
}


ETextureSourceFormat PixelFormatToSourceFormat(EPixelFormat PixelFormat)
{
	switch (PixelFormat)
	{
		case PF_R8_UINT:
			return TSF_G8;
		case PF_FloatRGBA:
			return TSF_RGBA16F;
		default:
			return TSF_Invalid;
	}
}

void CreatePersistentVolumeTexture(
	UVolumeTexture*& NewTexture, EPixelFormat NewTextureFormat, FIntVector NewTextureDimensions, uint8* RawDataArray)
{
	if(RawDataArray == nullptr)
		return;

	if (NewTextureDimensions.X == 0 && NewTextureDimensions.Y == 0 && NewTextureDimensions.Z == 0)
		return;
	
	if (!NewTexture->GetPlatformData())
	{
		NewTexture->SetPlatformData(new FTexturePlatformData());
	}

	NewTexture->GetPlatformData()->SizeX = NewTextureDimensions.X;
	NewTexture->GetPlatformData()->SizeY = NewTextureDimensions.Y;
	NewTexture->GetPlatformData()->SetNumSlices(NewTextureDimensions.Z);
	NewTexture->GetPlatformData()->PixelFormat = NewTextureFormat;

	NewTexture->SRGB = false;
	NewTexture->NeverStream = true;

	const long long TotalTextureSize = static_cast<long long>(NewTextureDimensions.X) * NewTextureDimensions.Y *
	                                   NewTextureDimensions.Z * GPixelFormats[NewTextureFormat].BlockBytes;

	NewTexture->GetPlatformData()->Mips.Add(new FTexture2DMipMap());
	auto Mip = NewTexture->GetPlatformData()->Mips.Last();
	Mip.SizeX = NewTextureDimensions.X;
	Mip.SizeY = NewTextureDimensions.Y;
	Mip.SizeZ = NewTextureDimensions.Z;

	Mip.BulkData.Lock(LOCK_READ_WRITE);

	uint8* ByteArray = Mip.BulkData.Realloc(TotalTextureSize);

	FMemory::Memcpy(ByteArray, RawDataArray, TotalTextureSize);

	Mip.BulkData.Unlock();

	NewTexture->MipGenSettings = TMGS_LeaveExistingMips;

	NewTexture->CompressionNone = true;

	const ETextureSourceFormat TextureSourceFormat = PixelFormatToSourceFormat(NewTextureFormat);

	if (TextureSourceFormat == TSF_Invalid)
	{
		return;
	}

	NewTexture->Source.Init(NewTextureDimensions.X, NewTextureDimensions.Y, NewTextureDimensions.Z, 1, TextureSourceFormat,
		RawDataArray);

	NewTexture->UpdateResource();
	NewTexture->MarkPackageDirty();
	FAssetRegistryModule::AssetCreated(NewTexture);

	NewTexture->UpdateResource();
}




UMaterialFunction* CreateLumaMaterialFunction(UVolumeTexture* T_O,
	UVolumeTexture* T_O_L, UVolumeTexture* T_F_1, UTexture2D* T_F_2, char* Luma_Src,
	FString MaterialName, FString PackageRoot = "")
{
	UMaterialFunction* UnrealMaterial = nullptr;
#ifdef WITH_EDITOR

	if (T_O == nullptr || T_O_L == nullptr || Luma_Src == nullptr)
	{
		return nullptr;
	}

	FString MaterialBaseName = "M_";
	MaterialBaseName += MaterialName;

	UnrealMaterial = CreateMaterialFunctionAssetPackage(MaterialBaseName, PackageRoot);

	UMaterialExpressionCustom* include_node = NewObject<UMaterialExpressionCustom>(UnrealMaterial);
	include_node->OutputType = CMOT_Float4;

	FString shader_src;
	
	TArray<FShaderCompilerError> errs;

	include_node->Code = "return float4(1, 1, 1, 1);}\n";
	include_node->Code += UTF8_TO_TCHAR(Luma_Src);
	include_node->Code += "\n void dummyFunc() {";

	UnrealMaterial->GetExpressionCollection().AddExpression(include_node);

	
	auto add_input = [](auto& node, auto input_name, auto& input_expression)
	{
		FExpressionInput input_exp;
		input_exp.Expression = input_expression;
		input_exp.InputName = input_name;
		node->Inputs.Add({input_name, input_exp});
	};

	auto add_standard_input = [&](auto& node, auto& input_expression)
	{
		FExpressionInput input_exp;
		input_exp.Expression = input_expression;
		node->Input = input_exp;
		UnrealMaterial->GetExpressionCollection().AddExpression(node);
	};

	auto add_ab_input = [&](auto& node, auto& input_expression_a, auto& input_expression_b)
	{
		FExpressionInput inp_node_A;
		inp_node_A.Expression = input_expression_a;
		node->A = inp_node_A;

		FExpressionInput inp_node_B;
		inp_node_B.Expression = input_expression_b;
		node->B = inp_node_B;
		UnrealMaterial->GetExpressionCollection().AddExpression(node);
	};

	auto add_a_input = [&](auto& node, auto& input_expression_a)
	{
		FExpressionInput inp_node_A;
		inp_node_A.Expression = input_expression_a;
		node->A = inp_node_A;
		UnrealMaterial->GetExpressionCollection().AddExpression(node);
	};

	
	UMaterialExpressionFunctionInput* crop_min_input = NewObject<UMaterialExpressionFunctionInput>(UnrealMaterial);
	crop_min_input->InputName = LumaConstants::CropMin;
	crop_min_input->UpdateMaterialExpressionGuid(true, true);
	crop_min_input->Id = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(crop_min_input);

	UMaterialExpressionFunctionInput* crop_max_input = NewObject<UMaterialExpressionFunctionInput>(UnrealMaterial);
	crop_max_input->InputName = LumaConstants::CropMax;
	crop_max_input->UpdateMaterialExpressionGuid(true, true);
	crop_max_input->Id = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(crop_max_input);

	UMaterialExpressionFunctionInput* crop_center_input = NewObject<UMaterialExpressionFunctionInput>(UnrealMaterial);
	crop_center_input->InputName = LumaConstants::CropCenter;
	crop_center_input->UpdateMaterialExpressionGuid(true, true);
	crop_center_input->Id = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(crop_center_input);

	UMaterialExpressionFunctionInput* crop_rotate_input = NewObject<UMaterialExpressionFunctionInput>(UnrealMaterial);
	crop_rotate_input->InputName = LumaConstants::CropRotation;
	crop_rotate_input->UpdateMaterialExpressionGuid(true, true);
	crop_rotate_input->Id = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(crop_rotate_input);

	UMaterialExpressionFunctionInput* quality_mode_input = NewObject<UMaterialExpressionFunctionInput>(UnrealMaterial);
	quality_mode_input->InputName = LumaConstants::QualityMode;
	quality_mode_input->UpdateMaterialExpressionGuid(true, true);
	quality_mode_input->Id = FGuid::NewGuid();
	quality_mode_input->InputType = EFunctionInputType::FunctionInput_Scalar;
	UnrealMaterial->GetExpressionCollection().AddExpression(quality_mode_input);

	UMaterialExpressionFunctionInput* exposure_input = NewObject<UMaterialExpressionFunctionInput>(UnrealMaterial);
	exposure_input->InputName = LumaConstants::Exposure;
	exposure_input->UpdateMaterialExpressionGuid(true, true);
	exposure_input->Id = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(exposure_input);


	// Set crop custom bock
	UMaterialExpressionCustom* shader_param_custom = NewObject<UMaterialExpressionCustom>(UnrealMaterial);
	shader_param_custom->OutputType = CMOT_Float1;
	FString crop_code = "";

	crop_code += "luma_output.crop_min = " + LumaConstants::CropMin.ToString() + ";\n";
	crop_code += "luma_output.crop_max = " + LumaConstants::CropMax.ToString() + ";\n";
	crop_code += "luma_output.crop_center =  " + LumaConstants::CropCenter.ToString() + ";\n";
	crop_code += "luma_output.crop_rotation =  " + LumaConstants::CropRotation.ToString() + ";\n";
	crop_code += "luma_output.exposure = " + LumaConstants::Exposure.ToString() + ";\n";

	// Quality settings
	crop_code += "switch (int(" + LumaConstants::QualityMode.ToString() + "))\n";
	crop_code += " { \n";

	for (int mode = 0; mode <= static_cast<int>(ELumaQualityMode::Extreme); mode++)
	{
		auto QualitySettings = get_luma_quality_settings(static_cast<ELumaQualityMode>(mode));
		crop_code += "case " + FString::FromInt(mode) + ": \n";
		crop_code += "luma_output.min_step_size = " + FString::SanitizeFloat(QualitySettings.min_step_size) + "; ";
		crop_code += "luma_output.lindisp_step = " + FString::SanitizeFloat(QualitySettings.lindisp_step) + "; ";
		crop_code += "luma_output.max_steps = " + FString::FromInt(QualitySettings.max_steps) + "; ";
		crop_code += "luma_output.expon_step = " + FString::FromInt(QualitySettings.expon_step) + "; ";
		crop_code += "luma_output.stop_thresh = " + FString::FromInt(QualitySettings.stop_thresh) + "; ";
		crop_code += "break;\n";
	}
	crop_code += " } \n";
	crop_code += "return 1.0f;";
	shader_param_custom->Code = crop_code;

	{
		shader_param_custom->Inputs.Empty();
		add_input(shader_param_custom, LumaConstants::CropMin, crop_min_input);
		add_input(shader_param_custom, LumaConstants::CropMax, crop_max_input);
		add_input(shader_param_custom, LumaConstants::CropCenter, crop_center_input);
		add_input(shader_param_custom, LumaConstants::CropRotation, crop_rotate_input);
		add_input(shader_param_custom, LumaConstants::QualityMode, quality_mode_input);
		add_input(shader_param_custom, LumaConstants::Exposure, exposure_input);
		add_input(shader_param_custom, "dummy", include_node);
		UnrealMaterial->GetExpressionCollection().AddExpression(shader_param_custom);
	}
	
	
	//pdo_custom->Inputs.Empty();

	UMaterialExpressionCustom* cube_setup_node = NewObject<UMaterialExpressionCustom>(UnrealMaterial);
	cube_setup_node->OutputType = CMOT_Float4;
	cube_setup_node->Code = R"UNQSTR(
		return PerformRaymarchCubeSetup(Parameters);
	)UNQSTR";

	FExpressionInput input_test;
	input_test.Expression = shader_param_custom;
	input_test.InputName = "dummy";
	cube_setup_node->Inputs.Empty();
	cube_setup_node->Inputs.Add({"dummy", input_test});
	UnrealMaterial->GetExpressionCollection().AddExpression(cube_setup_node);

	UMaterialExpressionComponentMask* mask_rgb_cube_setup = NewObject<UMaterialExpressionComponentMask>(UnrealMaterial);
	mask_rgb_cube_setup->R = 1;
	mask_rgb_cube_setup->G = 1;
	mask_rgb_cube_setup->B = 1;
	mask_rgb_cube_setup->A = 0;

	FExpressionInput mask_rgb_cube_input;
	mask_rgb_cube_input.Expression = cube_setup_node;
	mask_rgb_cube_setup->Input = mask_rgb_cube_input;

	UnrealMaterial->GetExpressionCollection().AddExpression(mask_rgb_cube_setup);

	UMaterialExpressionComponentMask* mask_a_cube_setup = NewObject<UMaterialExpressionComponentMask>(UnrealMaterial);
	mask_a_cube_setup->R = 0;
	mask_a_cube_setup->G = 0;
	mask_a_cube_setup->B = 0;
	mask_a_cube_setup->A = 1;

	FExpressionInput mask_a_cube_input;
	mask_a_cube_input.Expression = cube_setup_node;
	mask_a_cube_setup->Input = mask_a_cube_input;

	UnrealMaterial->GetExpressionCollection().AddExpression(mask_a_cube_setup);

	// Set up texture parameters
	UMaterialExpressionTextureObject* T_O_Node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);
	T_O_Node->Texture = T_O; 
	UnrealMaterial->GetExpressionCollection().AddExpression(T_O_Node);

	UMaterialExpressionTextureObject* T_O_L_Node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);
	T_O_L_Node->Texture = T_O_L; 
	UnrealMaterial->GetExpressionCollection().AddExpression(T_O_L_Node);

	UMaterialExpressionTextureObject* T_F_1_Node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);
	if (T_F_1)
	{
		T_F_1_Node->Texture = T_F_1;
	}
	else
	{
		T_F_1_Node->Texture = LoadObject<UVolumeTexture>(nullptr,
			TEXT("/Engine/EngineResources/DefaultVolumeTexture.DefaultVolumeTexture"), nullptr, LOAD_None, nullptr);
	}
	UnrealMaterial->GetExpressionCollection().AddExpression(T_F_1_Node);

	UMaterialExpressionTextureObject* T_F_2_Node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);

	if (T_F_2)
	{
		T_F_2_Node->Texture = T_F_2;
	}

	UnrealMaterial->GetExpressionCollection().AddExpression(T_F_2_Node);

	UMaterialExpressionTextureObject* dummy_tex_node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);

	UnrealMaterial->GetExpressionCollection().AddExpression(dummy_tex_node);

	auto create_scalar_param = [&](auto value, auto param_name)
	{
		UMaterialExpressionScalarParameter* new_param = NewObject<UMaterialExpressionScalarParameter>(UnrealMaterial);
		new_param->DefaultValue = value;
		new_param->ParameterName = param_name;
		UnrealMaterial->GetExpressionCollection().AddExpression(new_param);
		return new_param;
	};

	auto create_vector_param = [&](auto value, auto param_name)
	{
		UMaterialExpressionVectorParameter* new_param = NewObject<UMaterialExpressionVectorParameter>(UnrealMaterial);
		new_param->DefaultValue = value;
		new_param->ParameterName = param_name;
		UnrealMaterial->GetExpressionCollection().AddExpression(new_param);
		return new_param;
	};

	auto min_step_size_param = create_scalar_param(LumaConstants::MinStepSizeDefault, LumaConstants::MinStepSize);
	auto lindisp_step_param = create_scalar_param(LumaConstants::LindispStepDefault, LumaConstants::LindispStep);
	auto max_steps_param = create_scalar_param(LumaConstants::MaxStepsDefault, LumaConstants::MaxSteps);
	auto enable_inf_sample_param = create_scalar_param(LumaConstants::EnableInfSampleDefault, LumaConstants::EnableInfSample);

	auto dummy_scalar_node = create_scalar_param(42, "DummyScalar");

	UMaterialExpressionPixelDepth* pixel_depth_node = NewObject<UMaterialExpressionPixelDepth>(UnrealMaterial);

	UnrealMaterial->GetExpressionCollection().AddExpression(pixel_depth_node);

	// Create huge custom main node
	UMaterialExpressionCustom* raymarch_custom_node = NewObject<UMaterialExpressionCustom>(UnrealMaterial);
	raymarch_custom_node->OutputType = CMOT_Float4;

	FString main_raymarch_code = "";

	main_raymarch_code += "LumaRaymarch(";
	main_raymarch_code += LumaConstants::T_O.ToString() + ", ";
	main_raymarch_code += LumaConstants::T_O_L.ToString() + ", ";
	main_raymarch_code += LumaConstants::T_F_1.ToString() + ", ";
	main_raymarch_code += LumaConstants::T_F_1.ToString() + "Sampler, ";
	main_raymarch_code += LumaConstants::T_F_2.ToString() + ", ";
	main_raymarch_code += LumaConstants::EntryPos.ToString() + ", ";
	main_raymarch_code += LumaConstants::Thickness.ToString() + ", ";
	main_raymarch_code += LumaConstants::MinStepSize.ToString() + ", ";
	main_raymarch_code += LumaConstants::LindispStep.ToString() + ", ";
	main_raymarch_code += LumaConstants::MaxSteps.ToString() + ", ";
	main_raymarch_code += LumaConstants::EnableInfSample.ToString() + ", ";
	main_raymarch_code += LumaConstants::PixelDepth.ToString() + ", ";
	main_raymarch_code += "Parameters);\n";

	main_raymarch_code += "return luma_output.color;\n";

	raymarch_custom_node->Inputs.Empty();

	add_input(raymarch_custom_node, LumaConstants::T_O, T_O_Node);
	add_input(raymarch_custom_node, LumaConstants::T_O_L, T_O_L_Node);
	add_input(raymarch_custom_node, LumaConstants::T_F_1, T_F_1_Node);
	add_input(raymarch_custom_node, LumaConstants::T_F_2, T_F_2_Node);
	add_input(raymarch_custom_node, LumaConstants::EntryPos, mask_rgb_cube_setup);
	add_input(raymarch_custom_node, LumaConstants::Thickness, mask_a_cube_setup);
	add_input(raymarch_custom_node, LumaConstants::MinStepSize, min_step_size_param);
	add_input(raymarch_custom_node, LumaConstants::LindispStep, lindisp_step_param);
	add_input(raymarch_custom_node, LumaConstants::MaxSteps, max_steps_param);
	add_input(raymarch_custom_node, LumaConstants::EnableInfSample, enable_inf_sample_param);
	add_input(raymarch_custom_node, LumaConstants::PixelDepth, pixel_depth_node);

	raymarch_custom_node->Code = main_raymarch_code;

	UnrealMaterial->GetExpressionCollection().AddExpression(raymarch_custom_node);

	UMaterialExpressionComponentMask* mask_rgb_raymarch_output = NewObject<UMaterialExpressionComponentMask>(UnrealMaterial);
	mask_rgb_raymarch_output->R = 1;
	mask_rgb_raymarch_output->G = 1;
	mask_rgb_raymarch_output->B = 1;
	mask_rgb_raymarch_output->A = 0;

	add_standard_input(mask_rgb_raymarch_output, raymarch_custom_node);

	UMaterialExpressionFunctionOutput* color_output_node = NewObject<UMaterialExpressionFunctionOutput>(UnrealMaterial);
	color_output_node->OutputName = LumaConstants::MaterialFunctionColor;
	color_output_node->UpdateMaterialExpressionGuid(true, true);
	color_output_node->Id = FGuid::NewGuid();
	add_a_input(color_output_node, mask_rgb_raymarch_output);

	UMaterialExpressionComponentMask* mask_a_raymarch_output = NewObject<UMaterialExpressionComponentMask>(UnrealMaterial);
	mask_a_raymarch_output->R = 0;
	mask_a_raymarch_output->G = 0;
	mask_a_raymarch_output->B = 0;
	mask_a_raymarch_output->A = 1;

	add_standard_input(mask_a_raymarch_output, raymarch_custom_node);

	UMaterialExpressionConstant* zero_constant = NewObject<UMaterialExpressionConstant>(UnrealMaterial);
	zero_constant->R = 0.0f;
	UnrealMaterial->GetExpressionCollection().AddExpression(zero_constant);

	UMaterialExpressionConstant* half_constant = NewObject<UMaterialExpressionConstant>(UnrealMaterial);
	half_constant->R = 0.5f;
	UnrealMaterial->GetExpressionCollection().AddExpression(half_constant);


	UMaterialExpressionFunctionOutput* alpha_output_node = NewObject<UMaterialExpressionFunctionOutput>(UnrealMaterial);
	alpha_output_node->OutputName = LumaConstants::MaterialFunctionAlpha;
	alpha_output_node->UpdateMaterialExpressionGuid(true, true);
	alpha_output_node->Id = FGuid::NewGuid();
	add_a_input(alpha_output_node, mask_a_raymarch_output);

	UMaterialExpressionConstant3Vector* normal_constant = NewObject<UMaterialExpressionConstant3Vector>(UnrealMaterial);
	normal_constant->Constant = FLinearColor(0.0f, 0.0f, 1.0f);
	UnrealMaterial->GetExpressionCollection().AddExpression(normal_constant);

	UMaterialExpressionFunctionOutput* normal_output_node = NewObject<UMaterialExpressionFunctionOutput>(UnrealMaterial);
	normal_output_node->OutputName = LumaConstants::MaterialFunctionNormal;
	normal_output_node->UpdateMaterialExpressionGuid(true, true);
	normal_output_node->Id = FGuid::NewGuid();
	add_a_input(normal_output_node, normal_constant);

	// Pixel depth offset custom block
	UMaterialExpressionCustom* pdo_custom = NewObject<UMaterialExpressionCustom>(UnrealMaterial);
	pdo_custom->OutputType = CMOT_Float1;
	pdo_custom->Code = R"UNQSTR(
		return luma_output.pixel_depth_offset;
	)UNQSTR";

	pdo_custom->Inputs.Empty();

	add_input(pdo_custom, "dummy", raymarch_custom_node);

	UnrealMaterial->GetExpressionCollection().AddExpression(pdo_custom);

	UMaterialExpressionFunctionOutput* pdo_output_node = NewObject<UMaterialExpressionFunctionOutput>(UnrealMaterial);
	pdo_output_node->OutputName = LumaConstants::MaterialFunctionPDO;
	pdo_output_node->UpdateMaterialExpressionGuid(true, true);
	pdo_output_node->Id = FGuid::NewGuid();
	add_a_input(pdo_output_node, pdo_custom);


	// Build the point-to-plane functionality
	auto abs_world_pos_node = NewObject<UMaterialExpressionWorldPosition>(UnrealMaterial);
	UnrealMaterial->GetExpressionCollection().AddExpression(abs_world_pos_node);

	auto cam_pos_node = NewObject<UMaterialExpressionCameraPositionWS>(UnrealMaterial);
	UnrealMaterial->GetExpressionCollection().AddExpression(cam_pos_node);

	auto const_z_vector = NewObject<UMaterialExpressionConstant3Vector>(UnrealMaterial);
	const_z_vector->Constant = FLinearColor(0.0, 0.0, 1.0);
	UnrealMaterial->GetExpressionCollection().AddExpression(const_z_vector);

	auto cam_dir_node = NewObject<UMaterialExpressionTransform>(UnrealMaterial);
	cam_dir_node->TransformSourceType = TRANSFORMSOURCE_View;
	cam_dir_node->TransformType = TRANSFORM_World;

	add_standard_input(cam_dir_node, const_z_vector);
	auto near_plane_offset = NewObject<UMaterialExpressionCustom>(UnrealMaterial);
	near_plane_offset->OutputType = CMOT_Float1;
	near_plane_offset->Code = R"UNQSTR(
		return ResolvedView.NearPlane + 2.0f;
	)UNQSTR";
	near_plane_offset->Inputs.Empty();
	UnrealMaterial->GetExpressionCollection().AddExpression(near_plane_offset);

	// Scale X
	auto const_x_vector = NewObject<UMaterialExpressionConstant3Vector>(UnrealMaterial);
	const_x_vector->Constant = FLinearColor(1.0, 0.0, 0.0);
	UnrealMaterial->GetExpressionCollection().AddExpression(const_x_vector);

	auto transform_x_vector = NewObject<UMaterialExpressionTransform>(UnrealMaterial);
	transform_x_vector->TransformSourceType = TRANSFORMSOURCE_Instance;
	transform_x_vector->TransformType = TRANSFORM_World;

	add_standard_input(transform_x_vector, const_x_vector);

	// vector length
	auto scale_x_node = NewObject<UMaterialExpressionDistance>(UnrealMaterial);
	add_ab_input(scale_x_node, zero_constant, transform_x_vector);

	// Pivot point
	auto zero_vector_node = NewObject<UMaterialExpressionConstant3Vector>(UnrealMaterial);
	zero_vector_node->Constant = FLinearColor(0, 0, 0);
	UnrealMaterial->GetExpressionCollection().AddExpression(zero_vector_node);

	auto pivot_point_node = NewObject<UMaterialExpressionTransformPosition>(UnrealMaterial);
	pivot_point_node->TransformSourceType = TRANSFORMPOSSOURCE_Local;
	pivot_point_node->TransformType = TRANSFORMPOSSOURCE_World;

	add_standard_input(pivot_point_node, zero_vector_node);

	auto half_scale_node = NewObject<UMaterialExpressionDivide>(UnrealMaterial);
	half_scale_node->ConstB = 2.0f;
	{
		FExpressionInput inp_node_A;
		inp_node_A.Expression = scale_x_node;
		half_scale_node->A = inp_node_A;
		UnrealMaterial->GetExpressionCollection().AddExpression(half_scale_node);
	}

	auto subtract_pos_plane_pos_node = NewObject<UMaterialExpressionSubtract>(UnrealMaterial);
	add_ab_input(subtract_pos_plane_pos_node, cam_pos_node, abs_world_pos_node);

	auto dist_pos_plane_pos_node = NewObject<UMaterialExpressionDistance>(UnrealMaterial);
	add_ab_input(dist_pos_plane_pos_node, abs_world_pos_node, cam_pos_node);

	auto dist_pivot_plane_pos_node = NewObject<UMaterialExpressionDistance>(UnrealMaterial);
	add_ab_input(dist_pivot_plane_pos_node, cam_pos_node, pivot_point_node);

	auto normalize_sub_pos_node = NewObject<UMaterialExpressionNormalize>(UnrealMaterial);
	{
		FExpressionInput inp_node_A;
		inp_node_A.Expression = subtract_pos_plane_pos_node;
		normalize_sub_pos_node->VectorInput = inp_node_A;
		UnrealMaterial->GetExpressionCollection().AddExpression(normalize_sub_pos_node);
	}

	auto dot_normal_sub_pos_node = NewObject<UMaterialExpressionDotProduct>(UnrealMaterial);
	add_ab_input(dot_normal_sub_pos_node, cam_dir_node, normalize_sub_pos_node);

	auto multiply_dot_dist_node = NewObject<UMaterialExpressionMultiply>(UnrealMaterial);
	add_ab_input(multiply_dot_dist_node, dot_normal_sub_pos_node, dist_pos_plane_pos_node);

	auto lerp_dot_node = NewObject<UMaterialExpressionLinearInterpolate>(UnrealMaterial);
	lerp_dot_node->ConstA = 0.0f;
	lerp_dot_node->ConstAlpha = 9999.0f;
	{
		FExpressionInput inp_node_B;
		inp_node_B.Expression = dot_normal_sub_pos_node;
		lerp_dot_node->B = inp_node_B;
		UnrealMaterial->GetExpressionCollection().AddExpression(lerp_dot_node);
	}

	auto saturate_lerp_node = NewObject<UMaterialExpressionSaturate>(UnrealMaterial);
	add_standard_input(saturate_lerp_node, lerp_dot_node);

	auto multiply_normal_mult_node = NewObject<UMaterialExpressionMultiply>(UnrealMaterial);
	add_ab_input(multiply_normal_mult_node, cam_dir_node, multiply_dot_dist_node);

	auto multiply_sat_lerp_mult_node = NewObject<UMaterialExpressionMultiply>(UnrealMaterial);
	add_ab_input(multiply_sat_lerp_mult_node, saturate_lerp_node, multiply_normal_mult_node);

	auto multiply_normal_offset_node = NewObject<UMaterialExpressionMultiply>(UnrealMaterial);
	add_ab_input(multiply_normal_offset_node, cam_dir_node, near_plane_offset);

	auto add_mult_scalar_mult_node = NewObject<UMaterialExpressionAdd>(UnrealMaterial);
	add_ab_input(add_mult_scalar_mult_node, multiply_sat_lerp_mult_node, multiply_normal_offset_node);

	auto subtract_if_dist_node = NewObject<UMaterialExpressionSubtract>(UnrealMaterial);
	add_ab_input(subtract_if_dist_node, dist_pivot_plane_pos_node, half_scale_node);

	auto if_within_radius_node = NewObject<UMaterialExpressionIf>(UnrealMaterial);
	{
		FExpressionInput inp_node_A;
		inp_node_A.Expression = subtract_if_dist_node;
		if_within_radius_node->A = inp_node_A;

		FExpressionInput inp_node_B;
		inp_node_B.Expression = near_plane_offset;
		if_within_radius_node->B = inp_node_B;

		FExpressionInput inp_node_A_greater_than_B;
		inp_node_A_greater_than_B.Expression = zero_vector_node;
		if_within_radius_node->AGreaterThanB = inp_node_A_greater_than_B;

		FExpressionInput inp_node_A_less_than_B;
		inp_node_A_less_than_B.Expression = add_mult_scalar_mult_node;
		if_within_radius_node->ALessThanB = inp_node_A_less_than_B;
		if_within_radius_node->AEqualsB = inp_node_A_less_than_B;

		UnrealMaterial->GetExpressionCollection().AddExpression(if_within_radius_node);
	}

	UMaterialExpressionFunctionOutput* wpo_output_node = NewObject<UMaterialExpressionFunctionOutput>(UnrealMaterial);
	wpo_output_node->OutputName = LumaConstants::MaterialFunctionWPO;
	wpo_output_node->UpdateMaterialExpressionGuid(true, true);
	wpo_output_node->Id = FGuid::NewGuid();
	add_a_input(wpo_output_node, if_within_radius_node);

	UnrealMaterial->UpdateFromFunctionResource();
	UnrealMaterial->PreEditChange(nullptr);
	UnrealMaterial->PostEditChange();

#endif

	return UnrealMaterial;
}

UMaterial* CreateLumaMaterial(UMaterialFunction* MaterialFunction, TextureContainer& TexContainer,
	FString VolumeName, bool IsLit, FString PackageRoot = "")
{
	UMaterial* UnrealMaterial = nullptr;
#ifdef WITH_EDITOR

	if (!MaterialFunction)
	{
		return nullptr;
	}

	FString MaterialBaseName = "M_";
	MaterialBaseName += VolumeName;

	UnrealMaterial = CreateMaterialAssetPackage(MaterialBaseName, PackageRoot);

	UnrealMaterial->bTangentSpaceNormal = false;
	UnrealMaterial->BlendMode = BLEND_Masked;

	UMaterialEditorOnlyData* UnrealMaterialOutputEditorOnly = UnrealMaterial->GetEditorOnlyData();

	UMaterialExpressionMaterialFunctionCall* base_func_call_node = NewObject<UMaterialExpressionMaterialFunctionCall>(
		UnrealMaterial);
	UnrealMaterial->GetExpressionCollection().AddExpression(base_func_call_node);
	base_func_call_node->MaterialFunction = MaterialFunction;

	base_func_call_node->UpdateFromFunctionResource();

	auto func_outputs = base_func_call_node->FunctionOutputs;

	int idx = 0;
	for (const auto& func_output : func_outputs)
	{
		if (func_output.Output.OutputName == LumaConstants::MaterialFunctionColor)
		{
			UnrealMaterialOutputEditorOnly->BaseColor.Connect(idx, base_func_call_node);

			if (!IsLit)
			{
				UnrealMaterialOutputEditorOnly->EmissiveColor.Connect(idx, base_func_call_node);
			}
		}
		else if (func_output.Output.OutputName == LumaConstants::MaterialFunctionNormal)
		{
			UnrealMaterialOutputEditorOnly->Normal.Connect(idx, base_func_call_node);
		}
		else if (func_output.Output.OutputName == LumaConstants::MaterialFunctionAlpha)
		{
			UnrealMaterialOutputEditorOnly->OpacityMask.Connect(idx, base_func_call_node);
			UnrealMaterialOutputEditorOnly->Opacity.Connect(idx, base_func_call_node);
		}
		else if (func_output.Output.OutputName == LumaConstants::MaterialFunctionPDO)
		{
			UnrealMaterialOutputEditorOnly->PixelDepthOffset.Connect(idx, base_func_call_node);
		}
		else if (func_output.Output.OutputName == LumaConstants::MaterialFunctionWPO)
		{
			UnrealMaterialOutputEditorOnly->WorldPositionOffset.Connect(idx, base_func_call_node);
		}
		idx++;
	}

	// Add material param inputs

	UMaterialExpressionVectorParameter* crop_min_param = NewObject<UMaterialExpressionVectorParameter>(UnrealMaterial);
	crop_min_param->DefaultValue = FLinearColor(-32, -32, -32, -32);
	crop_min_param->ParameterName = LumaConstants::CropMin;
	crop_min_param->UpdateMaterialExpressionGuid(true, true);
	crop_min_param->ExpressionGUID = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(crop_min_param);

	UMaterialExpressionVectorParameter* crop_max_param = NewObject<UMaterialExpressionVectorParameter>(UnrealMaterial);
	crop_max_param->DefaultValue = FLinearColor(32, 32, 32, 32);
	crop_max_param->ParameterName = LumaConstants::CropMax;
	crop_max_param->UpdateMaterialExpressionGuid(true, true);
	crop_max_param->ExpressionGUID = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(crop_max_param);

	UMaterialExpressionVectorParameter* crop_center_param = NewObject<UMaterialExpressionVectorParameter>(UnrealMaterial);
	crop_center_param->DefaultValue = FLinearColor(0, 0, 0, 0);
	crop_center_param->ParameterName = LumaConstants::CropCenter;
	crop_center_param->UpdateMaterialExpressionGuid(true, true);
	crop_center_param->ExpressionGUID = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(crop_center_param);

	UMaterialExpressionVectorParameter* crop_rotate_param = NewObject<UMaterialExpressionVectorParameter>(UnrealMaterial);
	crop_rotate_param->DefaultValue = FLinearColor(0, 0, 0, 0);
	crop_rotate_param->ParameterName = LumaConstants::CropRotation;
	crop_rotate_param->UpdateMaterialExpressionGuid(true, true);
	crop_rotate_param->ExpressionGUID = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(crop_rotate_param);

	UMaterialExpressionScalarParameter* quality_mode_param = NewObject<UMaterialExpressionScalarParameter>(UnrealMaterial);
	quality_mode_param->DefaultValue = 0;
	quality_mode_param->ParameterName = LumaConstants::QualityMode;
	quality_mode_param->UpdateMaterialExpressionGuid(true, true);
	quality_mode_param->ExpressionGUID = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(quality_mode_param);
	
	UMaterialExpressionVectorParameter* exposure_param = NewObject<UMaterialExpressionVectorParameter>(UnrealMaterial);
	exposure_param->DefaultValue = FLinearColor(1, 1, 1, 1);
	exposure_param->ParameterName = LumaConstants::Exposure;
	exposure_param->UpdateMaterialExpressionGuid(true, true);
	exposure_param->ExpressionGUID = FGuid::NewGuid();
	UnrealMaterial->GetExpressionCollection().AddExpression(exposure_param);

	auto func_inputs = base_func_call_node->FunctionInputs;

	idx = 0;
	for (auto& func_input : func_inputs)
	{
		if (func_input.Input.InputName == LumaConstants::CropMin)
		{
			base_func_call_node->GetInput(idx)->Expression = crop_min_param;
			base_func_call_node->GetInput(idx)->Mask = crop_min_param->GetOutputs()[0].Mask;
			base_func_call_node->GetInput(idx)->MaskR = 1;
			base_func_call_node->GetInput(idx)->MaskG = 1;
			base_func_call_node->GetInput(idx)->MaskB = 1;
		}
		else if (func_input.Input.InputName == LumaConstants::CropMax)
		{
			base_func_call_node->GetInput(idx)->Expression = crop_max_param;
			base_func_call_node->GetInput(idx)->Mask = crop_max_param->GetOutputs()[0].Mask;
			base_func_call_node->GetInput(idx)->MaskR = 1;
			base_func_call_node->GetInput(idx)->MaskG = 1;
			base_func_call_node->GetInput(idx)->MaskB = 1;
		}
		else if (func_input.Input.InputName == LumaConstants::CropCenter)
		{
			base_func_call_node->GetInput(idx)->Expression = crop_center_param;
			base_func_call_node->GetInput(idx)->Mask = crop_center_param->GetOutputs()[0].Mask;
			base_func_call_node->GetInput(idx)->MaskR = 1;
			base_func_call_node->GetInput(idx)->MaskG = 1;
			base_func_call_node->GetInput(idx)->MaskB = 1;
		}
		else if (func_input.Input.InputName == LumaConstants::CropRotation)
		{
			base_func_call_node->GetInput(idx)->Expression = crop_rotate_param;
			base_func_call_node->GetInput(idx)->Mask = crop_rotate_param->GetOutputs()[0].Mask;
			base_func_call_node->GetInput(idx)->MaskR = 1;
			base_func_call_node->GetInput(idx)->MaskG = 1;
			base_func_call_node->GetInput(idx)->MaskB = 1;
		}
		else if (func_input.Input.InputName == LumaConstants::QualityMode)
		{
			base_func_call_node->GetInput(idx)->Expression = quality_mode_param;
			base_func_call_node->GetInput(idx)->Mask = quality_mode_param->GetOutputs()[0].Mask;
			base_func_call_node->GetInput(idx)->MaskR = 1;
		}
		else if (func_input.Input.InputName == LumaConstants::Exposure)
		{
			base_func_call_node->GetInput(idx)->Expression = exposure_param;
			base_func_call_node->GetInput(idx)->Mask = exposure_param->GetOutputs()[0].Mask;
			base_func_call_node->GetInput(idx)->MaskR = 1;
			base_func_call_node->GetInput(idx)->MaskG = 1;
			base_func_call_node->GetInput(idx)->MaskB = 1;
		}
		idx++;
	}

	base_func_call_node->UpdateFromFunctionResource();
	UMaterialExpressionConstant* zero_constant = NewObject<UMaterialExpressionConstant>(UnrealMaterial);
	zero_constant->R = 0.0f;
	UnrealMaterial->GetExpressionCollection().AddExpression(zero_constant);

	UnrealMaterialOutputEditorOnly->Metallic.Expression = zero_constant;
	UnrealMaterialOutputEditorOnly->Specular.Expression = zero_constant;

	UMaterialExpressionConstant* half_constant = NewObject<UMaterialExpressionConstant>(UnrealMaterial);
	half_constant->R = 0.5f;
	UnrealMaterial->GetExpressionCollection().AddExpression(half_constant);
	UnrealMaterialOutputEditorOnly->Roughness.Expression = half_constant;

	if (IsLit)
	{
		UnrealMaterial->SetShadingModel(MSM_DefaultLit);
	}
	else
	{
		UnrealMaterial->SetShadingModel(MSM_Unlit);
	}

	// UE requires texture references to exist, so we kind of have to repeat this here unfortunately

	if (auto cur_tex = TexContainer.T_O)
	{
		UMaterialExpressionTextureObject* dummy_node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);
		dummy_node->Texture = cur_tex;
		UnrealMaterial->GetExpressionCollection().AddExpression(dummy_node);
	}

	if (auto cur_tex = TexContainer.T_O_L)
	{
		UMaterialExpressionTextureObject* dummy_node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);
		dummy_node->Texture = cur_tex;
		UnrealMaterial->GetExpressionCollection().AddExpression(dummy_node);
	}
	if (auto cur_tex = TexContainer.T_S_O)
	{
		UMaterialExpressionTextureObject* dummy_node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);
		dummy_node->Texture = cur_tex;
		UnrealMaterial->GetExpressionCollection().AddExpression(dummy_node);
	}
	if (auto cur_tex = TexContainer.T_S_O_L)
	{
		UMaterialExpressionTextureObject* dummy_node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);
		dummy_node->Texture = cur_tex;
		UnrealMaterial->GetExpressionCollection().AddExpression(dummy_node);
	}
	if (auto cur_tex = TexContainer.T_F_2)
	{
		UMaterialExpressionTextureObject* dummy_node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);
		dummy_node->Texture = cur_tex;
		UnrealMaterial->GetExpressionCollection().AddExpression(dummy_node);
	}
	if (auto cur_tex = TexContainer.T_F_1)
	{
		UMaterialExpressionTextureObject* dummy_node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);
		dummy_node->Texture = cur_tex;
		UnrealMaterial->GetExpressionCollection().AddExpression(dummy_node);
	}

	UMaterialExpressionTextureObject* dummy_tex_node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);

	UnrealMaterial->GetExpressionCollection().AddExpression(dummy_tex_node);

	UMaterialExpressionTextureObject* dummy_vol_tex_node = NewObject<UMaterialExpressionTextureObject>(UnrealMaterial);
	dummy_vol_tex_node->Texture = LoadObject<UVolumeTexture>(nullptr,
		TEXT("/Engine/EngineResources/DefaultVolumeTexture.DefaultVolumeTexture"), nullptr, LOAD_None, nullptr);
	UnrealMaterial->GetExpressionCollection().AddExpression(dummy_vol_tex_node);

	UnrealMaterial->PreEditChange(nullptr);
	UnrealMaterial->PostEditChange();

#endif

	return UnrealMaterial;
}

UObject* CreateLumaBlueprint(UMaterial* Material, FName& BPName, float Scale, float CubeBorderScale, FString PackageRoot = "")
{
#ifdef WITH_EDITOR
	UPackage* BlueprintPackage = nullptr;
	UBlueprint* Blueprint = CreateLumaFieldAssetPackage(BPName.ToString(), PackageRoot, &BlueprintPackage);
	
	if (Blueprint)
	{
		// Editing the BluePrint
		ALumaField* BlueprintValues = Blueprint->GeneratedClass.Get()->GetDefaultObject<ALumaField>();
		BlueprintValues->StaticMeshComponent->SetRelativeScale3D(FVector(-Scale, Scale, Scale));
		BlueprintValues->StaticMeshComponent->SetRelativeRotation(FRotator3d::MakeFromEuler({90.0f, 0, 0}));

		if(CubeBorderScale < 1.0f)
		{
			// Segmented
			BlueprintValues->CropMin = {-2, -2, -2};
			BlueprintValues->CropMax = {2, 2, 2};
		}

		BlueprintValues->StaticMeshComponent->CastShadow = false;
		BlueprintValues->StaticMeshComponent->SetMaterial(0, Material);
	
		BlueprintValues->LumaBaseMaterial = Material;
		BlueprintValues->LumaMaterialInstance = UMaterialInstanceDynamic::Create((UMaterialInterface*) BlueprintValues->LumaBaseMaterial, (UObject*) Blueprint, FName("Luma Material Instance"));
	
		BlueprintValues->Version = ELumaVersion::LatestVersion;
		BlueprintValues->LoaderFlagsSet = true;

		// Compile the changes
		FCompilerResultsLog LogResults;
		FKismetEditorUtilities::CompileBlueprint(Blueprint, EBlueprintCompileOptions::None, &LogResults);

		// Notify the asset registry
		FAssetRegistryModule::AssetCreated(Blueprint);

		BlueprintPackage->FullyLoad();
		// Mark the package dirty...
		BlueprintPackage->MarkPackageDirty();
		BlueprintPackage->SetDirtyFlag(true);

		Blueprint->PreEditChange(nullptr);
		Blueprint->PostEditChange();
		return Blueprint;
	}
#endif
	return nullptr;
}

void GetValidPackageNameFromFileName(const FString& FullPath, FString& OutFilePath, FString& OutPackageName)
{
	FString ExtensionPart;

	FPaths::Split(FullPath, OutFilePath, OutPackageName, ExtensionPart);
	OutPackageName = FPaths::MakeValidFileName(OutPackageName);
	OutPackageName.ReplaceCharInline('.', '_');
}

class LStream : public std::stringbuf{
protected:
	int sync() {
		UE_LOG(LogTemp, Log, TEXT("%s"), *FString(str().c_str()));
		str("");
		return std::stringbuf::sync();
	}
};

UObject* ULumaLoader::CreateVolumeFromFileInExistingPackage(FString FileName, UObject* ParentPackage2)
{
#ifdef WITH_EDITOR

	FString FilePath, VolumeName;
	GetValidPackageNameFromFileName(FileName, FilePath, VolumeName);
	VolumeName = ObjectTools::SanitizeObjectName(VolumeName);
	auto guid = FGuid::NewGuid();
	FString AppendedVolumeName = VolumeName;
	FString GuidSuffix = guid.ToString().RightChop(30);
	AppendedVolumeName += "_" + GuidSuffix;
	int LastSlashIndex = -1;
	ParentPackage2->GetName().FindLastChar('/', LastSlashIndex);

	int ChopCount = ParentPackage2->GetName().Len() - LastSlashIndex - 1;

	FString PackageName;

	if (LastSlashIndex >= 0)
	{
		PackageName = (ParentPackage2->GetName().LeftChop(ChopCount) + "Luma_" + AppendedVolumeName + "/");
	}
	else
	{
		PackageName = (ParentPackage2->GetName() + "/Luma_" + AppendedVolumeName + "/");
	}

	LumaDLLLoader::LoadDLLHandle();
	LumaDLLLoader::LoadFunctions();
	LumaDLLLoader::LUMA_init();

	LStream Stream;
	std::cout.rdbuf(&Stream);

	LumaRenderSDK::LumaVolumeField* vol_field = LumaDLLLoader::LUMA_load_volume_field(TCHAR_TO_UTF8(*FileName));

	if (vol_field == nullptr)
	{
		FText error_title = FText::FromString("Error loading .luma file");
		FText error_message = FText::FromString("Error: Couldn't load .luma file. Please ensure that the filename only consists of latin characters, numbers and spaces!");
		const EAppReturnType::Type ReturnValue = FMessageDialog::Open(EAppMsgType::Ok, error_message, &error_title);

		return nullptr;
	}
	
	
	if (vol_field->version > LumaConstants::LumaShaderVersion)
	{
		FText error_title = FText::FromString("Luma field version newer than Plugin version");
		FText error_message = FText::FromString("Warning: The loaded Luma field contains features not supported by this version of the plugin. This can result in bugs, glitches or incorrect display. Please update the Luma UE plugin by downloading the new version on https://lumalabs.ai/ue !");
		const EAppReturnType::Type ReturnValue = FMessageDialog::Open(EAppMsgType::Ok, error_message, &error_title);
	}

	if(vol_field->F_1.data == nullptr)
	{
		FText error_title = FText::FromString("Luma field outdated");
		FText error_message = FText::FromString("Warning: The loaded Luma field was generated before UE-specific optimizations were added. Rendering performance is going to be *very* slow. Please join our discord at https://lumalabs.ai/discord to let us know and we can re-process your capture.");
		const EAppReturnType::Type ReturnValue = FMessageDialog::Open(EAppMsgType::Ok, error_message, &error_title);
	}

	// Create texture package
	FString TexturePackageRoot = PackageName + "Textures/";
	TextureContainer TexContainer;

	if (vol_field->T_S_O.data != nullptr)
	{
		TexContainer.T_S_O = CreateUVolumeTextureAssetPackage(VolumeName + "_Base_Segmented_" + GuidSuffix + "_" + LumaConstants::T_O.ToString(), TexturePackageRoot);
		CreatePersistentVolumeTexture(
		TexContainer.T_S_O, PF_R8_UINT, {vol_field->T_S_O.dims.x, vol_field->T_S_O.dims.y, vol_field->T_S_O.dims.z}, vol_field->T_S_O.data);
	}

	if (vol_field->T_S_O_L.data != nullptr)
	{
		TexContainer.T_S_O_L = CreateUVolumeTextureAssetPackage(VolumeName + "_Base_Segmented_" + GuidSuffix + "_" + LumaConstants::T_O_L.ToString(), TexturePackageRoot);
		CreatePersistentVolumeTexture(
		TexContainer.T_S_O_L, PF_R8_UINT, {vol_field->T_S_O_L.dims.x, vol_field->T_S_O_L.dims.y, vol_field->T_S_O_L.dims.z}, vol_field->T_S_O_L.data);
	}

	if (vol_field->T_O.data != nullptr)
	{
		TexContainer.T_O = CreateUVolumeTextureAssetPackage(VolumeName + "_Base_Full_" + GuidSuffix + "_" + LumaConstants::T_O.ToString(), TexturePackageRoot);
		CreatePersistentVolumeTexture(
		TexContainer.T_O, PF_R8_UINT, {vol_field->T_O.dims.x, vol_field->T_O.dims.y, vol_field->T_O.dims.z}, vol_field->T_O.data);
	}

	if (vol_field->T_O_L.data != nullptr)
	{
		TexContainer.T_O_L = CreateUVolumeTextureAssetPackage(VolumeName + "_Base_Full_" + GuidSuffix + "_" + LumaConstants::T_O_L.ToString(), TexturePackageRoot);
		CreatePersistentVolumeTexture(
		TexContainer.T_O_L, PF_R8_UINT, {vol_field->T_O_L.dims.x, vol_field->T_O_L.dims.y, vol_field->T_O_L.dims.z}, vol_field->T_O_L.data);
	}

	if (vol_field->F_1.data != nullptr)
	{
		TexContainer.T_F_1 = CreateUVolumeTextureAssetPackage(VolumeName + "_" + LumaConstants::T_F_1.ToString() + "_" + GuidSuffix, TexturePackageRoot);
		CreatePersistentVolumeTexture(
			TexContainer.T_F_1, PF_FloatRGBA, {vol_field->F_1.dims.x, vol_field->F_1.dims.y, vol_field->F_1.dims.z}, vol_field->F_1.data);
	}

	if (vol_field->F_2.data != nullptr)
	{
		TexContainer.T_F_2 = CreateUTexture2DAssetPackage(VolumeName + "_" + LumaConstants::T_F_2.ToString() + "_" + GuidSuffix, TexturePackageRoot);
		Setup2DTexture(
			TexContainer.T_F_2, PF_FloatRGBA, FIntPoint(vol_field->F_2.dims.x, vol_field->F_2.dims.y), vol_field->F_2.data);
	}
	
	const auto UnrealMaterialFunction = CreateLumaMaterialFunction(TexContainer.T_O, TexContainer.T_O_L, TexContainer.T_F_1, TexContainer.T_F_2, vol_field->shader, VolumeName + "_Base_Full_" + GuidSuffix, TexturePackageRoot);
	const auto UnrealMaterialFunctionBG = CreateLumaMaterialFunction(TexContainer.T_O, TexContainer.T_O_L, TexContainer.T_F_1, TexContainer.T_F_2, vol_field->shader_with_background,VolumeName + "_Base_Full_BG_" + GuidSuffix, TexturePackageRoot);
	const auto SegmentedUnrealMaterialFunction = CreateLumaMaterialFunction(TexContainer.T_S_O, TexContainer.T_S_O_L, TexContainer.T_F_1, TexContainer.T_F_2, vol_field->shader_segmented, VolumeName + "_Base_Segmented_" + GuidSuffix, TexturePackageRoot);

	const auto UnrealMaterial = CreateLumaMaterial(UnrealMaterialFunction, TexContainer, VolumeName + "_Full_Lit_" + GuidSuffix, true, TexturePackageRoot);
	const auto UnrealMaterialBG = CreateLumaMaterial(UnrealMaterialFunctionBG, TexContainer, VolumeName + "Full_Lit_BG_" + GuidSuffix, true, TexturePackageRoot);
	const auto SegmentedUnrealMaterial = CreateLumaMaterial(SegmentedUnrealMaterialFunction, TexContainer, 
		VolumeName + "_Segmented_Lit_" + GuidSuffix, true, TexturePackageRoot);
	const auto UnlitUnrealMaterial = CreateLumaMaterial(UnrealMaterialFunction, TexContainer, VolumeName + "_Full_Unlit_" + GuidSuffix,
		false, TexturePackageRoot);
	const auto UnlitUnrealMaterialBG = CreateLumaMaterial(UnrealMaterialFunctionBG, TexContainer, VolumeName + "_Full_Unlit_BG_" + GuidSuffix,
		false, TexturePackageRoot);
	const auto UnlitSegmentedUnrealMaterial = CreateLumaMaterial(SegmentedUnrealMaterialFunction, TexContainer,
		VolumeName + "_Segmented_Unlit_" + GuidSuffix, false, TexturePackageRoot);

	FGlobalComponentReregisterContext RecreateComponents1;

	FString BlueprintPackageRoot = PackageName + "Blueprints/";

	UObject* full_bp = nullptr;
	{
		FString BPBaseName = "BP_" + VolumeName + "_Dynamic";
		auto sanitized_name = ObjectTools::SanitizeObjectName(BPBaseName);
		FName BluePrintName = FName(sanitized_name);
		full_bp = CreateLumaBlueprint(UnrealMaterial, BluePrintName, (vol_field->T_O.dims.y / 4.0f) * 100,
			1.01f, BlueprintPackageRoot);
	}

	{
		FString BPBaseName = "BP_" + VolumeName + "_Dynamic_With_Environment";
		auto sanitized_name = ObjectTools::SanitizeObjectName(BPBaseName);
		FName BluePrintName = FName(sanitized_name);
		full_bp = CreateLumaBlueprint(UnrealMaterialBG, BluePrintName, (vol_field->T_O.dims.w / 4.0f) * 100,
			1.01f, BlueprintPackageRoot);
	}

	{
		FString BPBaseName = "BP_" + VolumeName + "_Dynamic_Cropped";
		auto sanitized_name = ObjectTools::SanitizeObjectName(BPBaseName);
		FName BluePrintName = FName(sanitized_name);

		if (vol_field->T_S_O.data)
		{
			float radius = exp2(float((vol_field->T_S_O.dims.z / vol_field->T_S_O.dims.w) - 1)) * 0.5f;
			CreateLumaBlueprint(SegmentedUnrealMaterial, BluePrintName,
				radius * 100, 0.505f, BlueprintPackageRoot);
		}
	}

	{
		FString BPBaseName = "BP_" + VolumeName + "_Baked";
		auto sanitized_name = ObjectTools::SanitizeObjectName(BPBaseName);
		FName BluePrintName = FName(sanitized_name);

		full_bp = CreateLumaBlueprint(UnlitUnrealMaterial, BluePrintName,
			(vol_field->T_O.dims.w / 4.0f) * 100, 1.01f, BlueprintPackageRoot);
	}

	{
		FString BPBaseName = "BP_" + VolumeName + "_Baked_With_Environment";
		auto sanitized_name = ObjectTools::SanitizeObjectName(BPBaseName);
		FName BluePrintName = FName(sanitized_name);

		full_bp = CreateLumaBlueprint(UnlitUnrealMaterialBG, BluePrintName,
			(vol_field->T_O.dims.w / 4.0f) * 100, 1.01f, BlueprintPackageRoot);
	}

	{
		FString BPBaseName = "BP_" + VolumeName + "_Baked_Cropped";
		auto sanitized_name = ObjectTools::SanitizeObjectName(BPBaseName);
		FName BluePrintName = FName(sanitized_name);

		if (vol_field->T_S_O.data)
		{
			float radius = exp2(float((vol_field->T_S_O.dims.z / vol_field->T_S_O.dims.w) - 1)) * 0.5f;
			CreateLumaBlueprint(UnlitSegmentedUnrealMaterial, BluePrintName,
				radius * 100, 0.505f, BlueprintPackageRoot);
		}
	}

	FGlobalComponentReregisterContext RecreateComponents;

	LumaDLLLoader::LUMA_cleanup();
	return full_bp;
#endif

	return nullptr;
}
