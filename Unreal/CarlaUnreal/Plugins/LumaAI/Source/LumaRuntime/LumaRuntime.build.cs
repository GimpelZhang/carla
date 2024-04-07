// # Copyright 2023 Luma AI
using UnrealBuildTool;

public class LumaRuntime: ModuleRules
{
	public LumaRuntime(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        CppStandard = CppStandardVersion.Cpp17;

        PublicIncludePaths.AddRange(
			new string[] {
				
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{ 
                "Core",
                "RenderCore",
                "Engine",
                "RHI",
                "AssetRegistry",
				"HeadMountedDisplay",
				"InputCore", 
				"Niagara",
				
			}
            );
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
                "SlateCore",
                "UMG",
                "Niagara",
                "Projects",
			}
            );
		
		if (Target.Type == TargetType.Editor) 
		{
			PrivateDependencyModuleNames.Add("UnrealEd");
		}
	}
}
