// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class Nav3D : ModuleRules
{
    public Nav3D(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        bUseUnity = true;

        // Ensure proper linking for Landscape module (UE 5.4 compatibility)
        bEnableUndefinedIdentifierWarnings = false;

        PublicIncludePaths.AddRange(
            new string[]
            {
                ModuleDirectory + "/../ThirdParty"
            }
        );

        PrivateIncludePaths.AddRange(
            new[]
            {
                "Nav3D/Private"
            }
        );

        PublicDependencyModuleNames.AddRange(
            new[]
            {
                "Core",
                "AIModule",
                "Landscape"  // MUST be public in UE 5.4 - GetPrivateStaticClass() requires public linkage
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new[]
            {
                "CoreUObject",
                "Engine",
                "Slate",
                "SlateCore",
                "RHI",
                "RenderCore",
                "DeveloperSettings",
                "GameplayTasks",
                "NavigationSystem",
                "InputCore",
            }
        );


        DynamicallyLoadedModuleNames.AddRange(
            new string[]
            {
				// ... add any modules that your module loads dynamically here ...
			}
        );

        if (Target.bBuildEditor)
            PublicDependencyModuleNames.Add("UnrealEd");
    }
}