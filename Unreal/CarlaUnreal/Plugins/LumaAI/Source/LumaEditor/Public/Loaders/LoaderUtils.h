// # Copyright 2023 Luma AI

#pragma once
namespace LoaderUtils
{
	UTexture2D* CreateUTexture2DAssetPackage(FString AssetName, FString PackageRoot);
	void Setup2DTexture(UTexture2D*& OutTexture, EPixelFormat PixelFormat, FIntPoint Dimensions, uint8* InSourceArray, TextureFilter Filter=TextureFilter::TF_Default);
	void GetValidPackageNameFromFileName(const FString& FullPath, FString& OutFilePath, FString& OutPackageName);

	void GetAssetAndPackageName(const FString& FileName, UObject* ParentPackage, FString& AssetName, FString& PackageName);
};