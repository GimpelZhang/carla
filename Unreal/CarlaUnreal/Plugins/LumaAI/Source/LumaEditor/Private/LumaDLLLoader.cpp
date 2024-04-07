// # Copyright 2023 Luma AI
#include "LumaDLLLoader.h"
#include "Interfaces/IPluginManager.h"

void LumaDLLLoader::FreeDLLHandle()
{
	if (DLLHandle != nullptr)
	{
		FPlatformProcess::FreeDllHandle(DLLHandle);
		DLLHandle = nullptr;
	}
}

bool LumaDLLLoader::LoadDLLHandle()
{
	// Don't load again.
	if (DLLHandle)
		return true;
	
	const FString BaseDir = IPluginManager::Get().FindPlugin("LumaAIPlugin")->GetBaseDir();
	const FString RenderSDKDir = FPaths::Combine(*BaseDir, TEXT("Source"), TEXT("LumaRenderSDK"));

	const FString LibDir = RenderSDKDir;
		
	FString Lib = FString(TEXT("LumaRenderSDK.")) + FPlatformProcess::GetModuleExtension();
	FString Path = LibDir.IsEmpty() ? *Lib : FPaths::Combine(*LibDir, *Lib);

	DLLHandle = FPlatformProcess::GetDllHandle(*Path);

	if (DLLHandle == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to load required library %s. Luma AI Plug-in will not be functional."), *Lib);
		return false;
	}
	return true;
}

bool LumaDLLLoader::LoadFunctions()
{
	LUMA_init = (LUMA_init_t) FPlatformProcess::GetDllExport(DLLHandle, TEXT("LUMA_init"));
	LUMA_load_splat_ply = (LUMA_load_splat_ply_t) FPlatformProcess::GetDllExport(DLLHandle, TEXT("LUMA_load_splat_ply"));
	LUMA_UE_chunk_gaussians = (LUMA_UE_chunk_gaussians_t) FPlatformProcess::GetDllExport(DLLHandle, TEXT("LUMA_UE_chunk_gaussians"));
	LUMA_load_volume_field = (LUMA_load_volume_field_t) FPlatformProcess::GetDllExport(DLLHandle, TEXT("LUMA_load_volume_field"));
	LUMA_cleanup = (LUMA_cleanup_t) FPlatformProcess::GetDllExport(DLLHandle, TEXT("LUMA_cleanup"));
	return true;
}
