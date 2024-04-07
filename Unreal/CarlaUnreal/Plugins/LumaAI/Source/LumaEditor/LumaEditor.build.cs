// # Copyright 2023 Luma AI

using UnrealBuildTool;
using System.IO;
public class LumaEditor : ModuleRules
{
	public LumaEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		AddEngineThirdPartyPrivateStaticDependencies(Target, "zlib");

		CppStandard = CppStandardVersion.Cpp17;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"UnrealEd", 
				"AssetTools",
				"Kismet",
				"Core",
				"RenderCore",
				"RHI",
				"AssetRegistry",
				"zlib",
			}
		);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore", 
				"LumaRuntime", 
				"Niagara",
                "Projects",
			}
		);
		
		// add DLL libraries
		string BaseDirectory = Path.GetFullPath(Path.Combine(ModuleDirectory, ".."));
		string RenderSDKDir = Path.Combine(BaseDirectory, "LumaRenderSDK");
		
		if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			RuntimeDependencies.Add(Path.Combine(RenderSDKDir, "LumaRenderSDK.so"));
		}
		else if (Target.Platform == UnrealTargetPlatform.Mac)
		{
			RuntimeDependencies.Add(Path.Combine(RenderSDKDir, "LumaRenderSDK.dylib"));
		}
		else if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			RuntimeDependencies.Add(Path.Combine(RenderSDKDir, "LumaRenderSDK.dll"));
		}

}
}