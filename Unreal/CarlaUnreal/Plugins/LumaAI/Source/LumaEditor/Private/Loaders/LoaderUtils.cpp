// # Copyright 2023 Luma AI

#include "Loaders/LoaderUtils.h"

#include "ObjectTools.h"
#include "AssetRegistry/AssetRegistryModule.h"
namespace LoaderUtils
{

UTexture2D* CreateUTexture2DAssetPackage(FString AssetName, FString PackageRoot)
{
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

void Setup2DTexture(UTexture2D*& OutTexture, EPixelFormat PixelFormat, FIntPoint Dimensions, uint8* InSourceArray, TextureFilter Filter)
{
	// Newly created Volume textures have this null'd
	if (!OutTexture->GetPlatformData())
	{
		OutTexture->SetPlatformData(new FTexturePlatformData());
	}
	// Set Dimensions and Pixel format.

	auto PlatformData = OutTexture->GetPlatformData();
	PlatformData->SizeX = Dimensions.X;
	PlatformData->SizeY = Dimensions.Y;
	PlatformData->SetNumSlices(1);
	PlatformData->PixelFormat = PixelFormat;
	OutTexture->Filter = Filter;
	OutTexture->CompressionSettings = TC_HDR;
	OutTexture->SRGB = false;
	OutTexture->NeverStream = true;

	// Create texture
	int BlockBytes = GPixelFormats[PixelFormat].BlockBytes;
	int TotalBytes = Dimensions.X * Dimensions.Y * BlockBytes;
	PlatformData->Mips.Add(new FTexture2DMipMap());
	auto Mip = PlatformData->Mips.Last();
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

void GetValidPackageNameFromFileName(const FString& FullPath, FString& OutFilePath, FString& OutPackageName)
{
	FString ExtensionPart;

	FPaths::Split(FullPath, OutFilePath, OutPackageName, ExtensionPart);
	OutPackageName = FPaths::MakeValidFileName(OutPackageName);
	OutPackageName.ReplaceCharInline('.', '_');
}

void GetAssetAndPackageName(const FString& FileName, UObject* ParentPackage, FString& AssetName, FString& PackageName)
{
	FString FilePath;
	LoaderUtils::GetValidPackageNameFromFileName(FileName, FilePath, AssetName);
	AssetName = ObjectTools::SanitizeObjectName(AssetName);
	auto guid = FGuid::NewGuid();
	FString GuidSuffix = guid.ToString().RightChop(30);
	AssetName += "_" + GuidSuffix;
	int LastSlashIndex = -1;
	ParentPackage->GetName().FindLastChar('/', LastSlashIndex);

	int ChopCount = ParentPackage->GetName().Len() - LastSlashIndex - 1;

	if (LastSlashIndex >= 0)
	{
		PackageName = (ParentPackage->GetName().LeftChop(ChopCount) + "Luma_" + AssetName + "/");
	}
	else
	{
		PackageName = (ParentPackage->GetName() + "/Luma_" + AssetName + "/");
	}

}
};
