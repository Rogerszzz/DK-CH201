[Version]
Class=IEXPRESS
SEDVersion=3

[Options]
ExtractOnly=1
PackagePurpose=ExtractOnly
ShowInstallProgramWindow=0
HideExtractAnimation=0
UseLongFileName=1
InsideCompressed=0
CAB_FixedSize=0
CAB_ResvCodeSigning=0
RebootMode=I
InstallPrompt=%InstallPrompt%
DisplayLicense=%DisplayLicense%
FinishMessage=%FinishMessage%
TargetName=%TargetName%
FriendlyName=%FriendlyName%
AppLaunched=%AppLaunched%
PostInstallCmd=%PostInstallCmd%
AdminQuietInstCmd=%AdminQuietInstCmd%
UserQuietInstCmd=%UserQuietInstCmd%
SourceFiles=SourceFiles

[SourceFiles]
SourceFiles0=.

[SourceFiles0]
%FILE0%=

[Strings]
InstallPrompt=
DisplayLicense=../LICENSE.txt
FinishMessage=Thank you for installing Chirp SonicLib. Please navigate to the installation directory and unzip the package.
AppLaunched=
PostInstallCmd=
AdminQuietInstCmd=
UserQuietInstCmd=
