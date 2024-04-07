// # Copyright 2023 Luma AI

#include "Loaders/LumaSplatLoader.h"
#ifdef WITH_EDITOR
#include "AssetRegistry/AssetRegistryModule.h"
#include "Kismet2/CompilerResultsLog.h"
#include "Kismet2/KismetEditorUtilities.h"
#endif
#include "LumaScene.h"
#include "NiagaraSystem.h"
#include "NiagaraComponent.h"
#include "Loaders/RaymarchMaterialParameters.h"
#include "Loaders/LoaderUtils.h"
#include "Async/ParallelFor.h"
#include "Misc/ScopedSlowTask.h"
#include "LumaDLLLoader.h"
#include "LumaRenderSDKPublic.h"

ULumaSplatLoader* ULumaSplatLoader::Get()
{
	return NewObject<ULumaSplatLoader>();
}

template<typename F>
TArray<FFloat16> InitializeF16GaussianTextureDataPerTexel(uint32 texture_width, uint32 texture_height, F && per_texel_lambda)
{

	TArray<FFloat16> texture_data;

	uint32 num_filled_texels = 0;
	uint32 iteration_index = 0;

	uint32 num_texels = texture_width * texture_height;

	while(num_filled_texels < num_texels)
	{
		int success = per_texel_lambda(iteration_index, texture_data);

		if (success == 1)
		{
			num_filled_texels++;
		}

		if (success == 2)
		{
			break;
		}

		iteration_index++;
	}

	while(num_filled_texels < num_texels)
	{
		texture_data.Add(0);
		texture_data.Add(0);
		texture_data.Add(0);
		texture_data.Add(0);

		num_filled_texels++;
	}
	
	for (uint32 x = 0; x < texture_width; x++)
	{
		for (uint32 y = 0; y < texture_height; y++)
		{
			uint32 flat_index = y + texture_height * x;
			per_texel_lambda(flat_index, texture_data);
		}
	}

	
	return texture_data;
}

UBlueprint* CreateLumaSceneAssetPackage(FString AssetName, FString PackageRoot, UPackage** BlueprintPackage, UClass* BPClass)
{
	FString PackageName = PackageRoot + AssetName;
	*BlueprintPackage = CreatePackage( *PackageName);
	UBlueprint* UnrealObject = FKismetEditorUtilities::CreateBlueprint(BPClass, *BlueprintPackage, *AssetName,
		BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());

	return UnrealObject;
}

void CreateGaussianChunkLuma(const LumaRenderSDK::Gaussians& gaussians, const LumaRenderSDK::GaussianChunk& chunk, FGaussianComponentChunk& ret)
{
	ret.col = nullptr;
	ret.pos = nullptr;
	ret.rot = nullptr;
	ret.scale = nullptr;
	ret.extent = {chunk.extent.x, chunk.extent.y, chunk.extent.z};
	ret.center = {chunk.center.x, chunk.center.y, chunk.center.z};

	FString chunk_suffix = ret.center.ToString() + "_" + ret.extent.ToString();

	chunk_suffix = FMD5::HashBytes((unsigned char*)chunk_suffix.GetCharArray().GetData(), chunk_suffix.Len()).Left(8);
	
	uint32 num_gaussians = gaussians.num;

	if(chunk.num_gaussians > 100)
		num_gaussians = chunk.num_gaussians;
	else if(chunk.indices != nullptr)
	{
		// Early out if num gaussians is too low
		ret.num = 0;
		return;
	}
		
	ret.num = num_gaussians;

	// Texture size
	int gaussian_texture_width = 2048;
	int gaussian_texture_height = FMath::Max(FMath::RoundUpToPowerOfTwo( num_gaussians / (float)gaussian_texture_width ), 4U);

	if(num_gaussians < 1024 * 512)
	{
		gaussian_texture_width = FMath::RoundUpToPowerOfTwo(FMath::Sqrt((float)num_gaussians));
		gaussian_texture_height = gaussian_texture_width;
	}

	auto texture_width = gaussian_texture_width;
	auto texture_height = gaussian_texture_height;
	ret.texture_width = texture_width;
	ret.texture_height = texture_height;
	
	ret.pos_data = InitializeF16GaussianTextureDataPerTexel(texture_width, texture_height,
		[&] (uint32 flat_index, TArray<FFloat16>& texture_data)
		{			
			if (flat_index >= num_gaussians)
			{
				// Break out of the loop
				return 2;
			}

			uint32 gaussian_index = flat_index;
			
			if(chunk.num_gaussians != 0 && chunk.indices != nullptr)
				gaussian_index = chunk.indices[flat_index];

			auto& gaussian = gaussians.data[gaussian_index];

			FVector3f gauss_pos = {gaussian.pos.x, -gaussian.pos.z, -gaussian.pos.y};
			
			
			FVector4f Pos(gauss_pos.X / 32.0f, gauss_pos.Y / 32.0f, gauss_pos.Z / 32.0f, 0.0f);

			texture_data.Add(Pos.X);
			texture_data.Add(Pos.Y);
			texture_data.Add(Pos.Z);
			texture_data.Add(Pos.W);

			return 1;
		}
	);

	ret.col_data = InitializeF16GaussianTextureDataPerTexel(texture_width, texture_height,
		[&](uint32 flat_index, TArray<FFloat16>& texture_data)
		{
			if (flat_index >= num_gaussians)
			{
				// Break out of the loop
				return 2;
			}

			uint32 gaussian_index = flat_index;
			
			if(chunk.num_gaussians != 0 && chunk.indices != nullptr)
				gaussian_index = chunk.indices[flat_index];

			auto& gaussian = gaussians.data[gaussian_index];

			FVector4f Color{gaussian.color.x, gaussian.color.y, gaussian.color.z, gaussian.color.w};

			texture_data.Add(Color.X);
			texture_data.Add(Color.Y);
			texture_data.Add(Color.Z);
			texture_data.Add(Color.W);

			return 1;
		}
		);

	ret.rot_data = InitializeF16GaussianTextureDataPerTexel(texture_width, texture_height,
		[&] (uint32 flat_index, TArray<FFloat16>& texture_data)
		{
			if (flat_index >= num_gaussians)
			{
				// Break out of the loop
				return 2;
			}

			uint32 gaussian_index = flat_index;
			
			if(chunk.num_gaussians != 0 && chunk.indices != nullptr)
				gaussian_index = chunk.indices[flat_index];

			auto& gaussian = gaussians.data[gaussian_index];
			
			FVector4f Rot{gaussian.rotation.x, gaussian.rotation.y, gaussian.rotation.z, gaussian.rotation.w};
			
			texture_data.Add(Rot.X);
			texture_data.Add(Rot.Y);
			texture_data.Add(Rot.Z);
			texture_data.Add(Rot.W);

			return 1;
		}
	);

	ret.scale_data = InitializeF16GaussianTextureDataPerTexel(texture_width, texture_height,
		[&] (uint32 flat_index, TArray<FFloat16>& texture_data)
		{
			if (flat_index >= num_gaussians)
			{
				// Break out of the loop
				return 2;
			}

			uint32 gaussian_index = flat_index;
			
			if(chunk.num_gaussians != 0 && chunk.indices != nullptr)
				gaussian_index = chunk.indices[flat_index];

			auto& gaussian = gaussians.data[gaussian_index];
			
			FVector4f Rot(gaussian.scale.x / 32.0f, gaussian.scale.y / 32.0f, gaussian.scale.z / 32.0f, 1.0f);
			
			texture_data.Add(Rot.X);
			texture_data.Add(Rot.Y);
			texture_data.Add(Rot.Z);
			texture_data.Add(Rot.W);

			return 1;
		}
	);
}

struct MaterialPermutation
{
	FString suffix;
	UMaterial* mat;
};

struct RTVisiblePermutaiton
{
	FString suffix;
	bool rt_visible;
};

UObject* ULumaSplatLoader::CreateFromFileInExistingPackage(FString FileName, UObject* ParentPackage)
{
	LumaDLLLoader::LoadDLLHandle();
	LumaDLLLoader::LoadFunctions();
	LumaDLLLoader::LUMA_init();

	LumaRenderSDK::Gaussians* gaussians = LumaDLLLoader::LUMA_load_splat_ply(TCHAR_TO_UTF8(*FileName));
	
	FScopedSlowTask LumaProgressBar(110.0f, FText::FromString("Loading Luma Interactive Scene..."));
	LumaProgressBar.MakeDialog();
	LumaProgressBar.EnterProgressFrame(50.0f);

	FString AssetName;
	FString PackageName;

	LoaderUtils::GetAssetAndPackageName(FileName, ParentPackage, AssetName, PackageName);

	UNiagaraSystem * NS = LoadObject<UNiagaraSystem>(nullptr, TEXT("/Script/Niagara.NiagaraSystem'/LumaAIPlugin/Blueprints/LumaInteractiveSceneSystem.LumaInteractiveSceneSystem'"), nullptr, LOAD_None, nullptr);

	UPackage* BlueprintPackage = nullptr;

	// Get baked and dynamic materials
	TArray<MaterialPermutation> mats = {
		{"_Baked_No_TAA", LoadObject<UMaterial>(nullptr, TEXT("/Script/Engine.Material'/LumaAIPlugin/Materials/LumaMaterialBakedNoTAA.LumaMaterialBakedNoTAA'"), nullptr, LOAD_None, nullptr)},
		{"_Dynamic", LoadObject<UMaterial>(nullptr, TEXT("/Script/Engine.Material'/LumaAIPlugin/Materials/LumaMaterialDynamic.LumaMaterialDynamic'"), nullptr, LOAD_None, nullptr)},
		{"_Dynamic_No_TAA", LoadObject<UMaterial>(nullptr, TEXT("/Script/Engine.Material'/LumaAIPlugin/Materials/LumaMaterialDynamicNoTAA.LumaMaterialDynamicNoTAA'"), nullptr, LOAD_None, nullptr)},
		{"_Baked", LoadObject<UMaterial>(nullptr, TEXT("/Script/Engine.Material'/LumaAIPlugin/Materials/LumaMaterialBaked.LumaMaterialBaked'"), nullptr, LOAD_None, nullptr)},
	};

	TArray<RTVisiblePermutaiton> rt_configs = {
		{"", false}
	};

	int num_chunks = 1;
	int num_chunks_per_dim = FMath::Pow(ALumaScene::NUM_NGC_COMPONENTS + 1, 1.0 / 3.0);
	if (gaussians->num > LumaConstants::MaxNumGaussiansSingleChunk)
	{
		num_chunks = ALumaScene::NUM_NGC_COMPONENTS;
	}

	auto create_texture_asset = [&] (FString TextureName, FString TexturePackage, uint32 texture_width, uint32 texture_height, TArray<FFloat16> texture_data, UTexture2D** tex)
	{
		auto texture_format = PF_FloatRGBA;
		*tex = LoaderUtils::CreateUTexture2DAssetPackage(TextureName, TexturePackage);
		LoaderUtils::Setup2DTexture(
			*tex, texture_format, FIntPoint(texture_width, texture_height), reinterpret_cast<uint8*>(texture_data.GetData()), TextureFilter::TF_Nearest);
	};
	
	auto set_ngc = [&] (FGaussianComponentChunk& gchunk, UNiagaraComponent* ngc, UNiagaraSystem* NS, int32 index, UMaterial* Material, bool rt_visible)
	{
		ngc->SetAsset(NS);
		ngc->SetVariableFloat(LumaConstants::GSNumGaussians, gchunk.num);
		ngc->SetVariableTexture(LumaConstants::GSColorTex, gchunk.col);
		ngc->SetVariableTexture(LumaConstants::GSPosTex, gchunk.pos);
		ngc->SetVariableTexture(LumaConstants::GSRotTex, gchunk.rot);
		ngc->SetVariableTexture(LumaConstants::GSScaleTex, gchunk.scale);
		ngc->SetVariableMaterial(LumaConstants::GSMaterial, Material);
		ngc->SetAutoActivate(true);

		const FVector chunk_center_world = FVector{gchunk.center.X * 100.0f, gchunk.center.Y * 100.0f, gchunk.center.Z * 100.0f};
		
		ngc->SetVariableVec3(LumaConstants::GSChunkOffset, chunk_center_world);
		ngc->SetVariableVec2(LumaConstants::GSTextureSize, {(float)gchunk.texture_width, (float)gchunk.texture_height});
		ngc->SetVariableVec2(LumaConstants::GSInvTextureSize, {1.0f / (gchunk.texture_width + 0.5f), 1.0f / gchunk.texture_height});
		ngc->Activate(true);
		ngc->SetAllPhysicsPosition(chunk_center_world);
		ngc->SetVisibleInRayTracing(rt_visible);
	};

	ALumaScene* BlueprintValues = nullptr;

	// Get min/max with coordinate system adjusted
	FVector3f min_pos = {999999, 999999, 999999};
	FVector3f max_pos = {-999999, -999999, -999999};
	for(uint32 gi = 0; gi < gaussians->num; gi++)
	{
		auto& gaussian = gaussians->data[gi];
		FVector3f p = {gaussian.pos.y, -gaussian.pos.z, -gaussian.pos.y};
		min_pos = FVector3f::Min(p, min_pos);
		max_pos = FVector3f::Max(p, max_pos);
	}

	// extend bounds slightly so we won't run into issues later
	min_pos = min_pos - 0.1f;
	max_pos = max_pos + 0.1f;
	
	UBlueprint* Blueprint = nullptr;
	TArray<FGaussianComponentChunk> gchunks;
	gchunks.SetNum(num_chunks);

	LumaRenderSDK::GaussianChunks* gaussian_chunks = LumaDLLLoader::LUMA_UE_chunk_gaussians(gaussians);
	LumaProgressBar.EnterProgressFrame(10.0f);

	ParallelFor(num_chunks, [&](int32 Idx) {
		int x = Idx / (num_chunks_per_dim * num_chunks_per_dim);
		int y = (Idx / (num_chunks_per_dim)) % num_chunks_per_dim;
		int z = Idx % num_chunks_per_dim;

		int flat_index = x + y * num_chunks_per_dim + z * num_chunks_per_dim * num_chunks_per_dim;

		CreateGaussianChunkLuma(*gaussians, gaussian_chunks->chunks[flat_index], gchunks[flat_index]);
	});
	LumaProgressBar.EnterProgressFrame(30.0f);
	for(int i = 0; i < num_chunks; i++)
	{
		auto& gchunk = gchunks[i];

		if (gchunk.num == 0)
		{
			continue;
		}

		create_texture_asset(AssetName + LumaConstants::GSColorTex.ToString() + FString::FromInt(i), PackageName + "Textures/", gchunk.texture_width, gchunk.texture_height, gchunk.col_data, reinterpret_cast<UTexture2D**>(&gchunk.col));
		create_texture_asset(AssetName + LumaConstants::GSPosTex.ToString() + FString::FromInt(i), PackageName + "Textures/", gchunk.texture_width, gchunk.texture_height, gchunk.pos_data, reinterpret_cast<UTexture2D**>(&gchunk.pos));
		create_texture_asset(AssetName + LumaConstants::GSRotTex.ToString() + FString::FromInt(i), PackageName + "Textures/", gchunk.texture_width, gchunk.texture_height, gchunk.rot_data, reinterpret_cast<UTexture2D**>(&gchunk.rot));
		create_texture_asset(AssetName + LumaConstants::GSScaleTex.ToString() + FString::FromInt(i), PackageName + "Textures/", gchunk.texture_width, gchunk.texture_height, gchunk.scale_data, reinterpret_cast<UTexture2D**>(&gchunk.scale));
	}

	LumaProgressBar.EnterProgressFrame(15.0f);

	auto create_bp = [&] (const MaterialPermutation& mat, const RTVisiblePermutaiton& rt)
	{

		if(num_chunks != 1)
		{
			Blueprint = CreateLumaSceneAssetPackage(AssetName + rt.suffix + mat.suffix, PackageName, &BlueprintPackage, ALumaScene::StaticClass());
			BlueprintValues = Blueprint->GeneratedClass.Get()->GetDefaultObject<ALumaScene>();
		}
		else
		{
			Blueprint = CreateLumaSceneAssetPackage(AssetName + rt.suffix + mat.suffix, PackageName, &BlueprintPackage, ALumaSceneSingle::StaticClass());
			BlueprintValues = Blueprint->GeneratedClass.Get()->GetDefaultObject<ALumaSceneSingle>();
		}
	
		for(int i = 0; i < num_chunks; i++)
		{
			auto& gchunk = gchunks[i];

			if (gchunk.num == 0)
			{
				BlueprintValues->LumaNiagaraComponents[i]->DestroyInstance();
				BlueprintValues->LumaNiagaraComponents[i] = nullptr;
				continue;
			}

			auto ngc = BlueprintValues->LumaNiagaraComponents[i];
			set_ngc(gchunk, ngc, NS, i, mat.mat, rt.rt_visible);
		}

		// Remove null entries from list
		int i = 0;
		while(i < BlueprintValues->LumaNiagaraComponents.Num())
		{
			if(BlueprintValues->LumaNiagaraComponents[i] == nullptr)
			{
				BlueprintValues->LumaNiagaraComponents.RemoveAt(i);
			}
			else
			{
				i++;
			}
		}

		BlueprintValues->Version = ELumaVersion::LatestVersion;
	
		// Compile the changes
		FCompilerResultsLog LogResults;
		FKismetEditorUtilities::CompileBlueprint(Blueprint, EBlueprintCompileOptions::None, &LogResults);

		// Notify the asset registry
		FAssetRegistryModule::AssetCreated(Blueprint);
	
		Blueprint->PreEditChange(nullptr);
		Blueprint->PostEditChange();

	};
	
	// Create blueprints
	for(const auto& mat_type : mats)
	{
		for(const auto& rt_type : rt_configs)
		{
			create_bp(mat_type, rt_type);
		}
	}

	LumaProgressBar.EnterProgressFrame(5.0f);
	
	BlueprintPackage->FullyLoad();
	// Mark the package dirty...
	BlueprintPackage->MarkPackageDirty();
	BlueprintPackage->SetDirtyFlag(true);
	LumaDLLLoader::LUMA_cleanup();
	return Blueprint;
}

