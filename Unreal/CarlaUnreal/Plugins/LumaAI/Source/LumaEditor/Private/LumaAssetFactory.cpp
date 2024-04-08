// # Copyright 2023 Luma AI
#include "LumaAssetFactory.h"
#include "Loaders/LumaLoader.h"
#include "Loaders/LumaSplatLoader.h"

ULumaAssetFactory::ULumaAssetFactory(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	Formats.Add(FString(TEXT("luma;")) + NSLOCTEXT("LumaAssetFactory", "FormatLuma", ".luma File").ToString());
	Formats.Add(FString(TEXT("ply;")) + NSLOCTEXT("LumaAssetFactory", "FormatLumaPLY", ".ply File").ToString());

	SupportedClass = UObject::StaticClass();
	bCreateNew = false;
	bEditorImport = true;
	ImportPriority = 1;
}

UObject* ULumaAssetFactory::FactoryCreateFile(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags,
	const FString& Filename, const TCHAR* Parms, FFeedbackContext* Warn, bool& bOutOperationCanceled)
{
	FString FileNamePart, FolderPart, ExtensionPart;
	FPaths::Split(Filename, FolderPart, FileNamePart, ExtensionPart);

	UObject* OutObject = nullptr;

	if (ExtensionPart == "luma")
	{
		const auto Loader = ULumaLoader::Get();

		OutObject = Loader->CreateVolumeFromFileInExistingPackage(Filename, InParent);
		bOutOperationCanceled = false;
	}
	else if (ExtensionPart == "ply")
	{
		const auto Loader = ULumaSplatLoader::Get();

		OutObject = Loader->CreateFromFileInExistingPackage(Filename, InParent);
		bOutOperationCanceled = false;
	}

	return OutObject;
}
