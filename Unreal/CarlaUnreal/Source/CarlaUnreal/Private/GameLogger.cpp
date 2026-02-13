#include "GameLogger.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/DateTime.h"
#include "TimerManager.h"

UGameLogger::UGameLogger()
{
    // Empty constructor
}

void UGameLogger::InitializeLogger(const FString& UserID, const FString& ScN, const FString& Displ, const FString& AbsoluteFolderPath)
{
    IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

    // Ensure the folder path is treated as absolute
    LogDirectory = AbsoluteFolderPath + "/" + UserID;

    // Check if directory exists, create it if necessary
    if (!PlatformFile.DirectoryExists(*LogDirectory))
    {
        bool bSuccess = PlatformFile.CreateDirectoryTree(*LogDirectory);
        if (!bSuccess)
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to create log directory: %s"), *LogDirectory);
            return; // Stop execution if folder creation fails
        }
    }

    // Generate a safe filename with timestamp
    FString Timestamp = FDateTime::Now().ToString(TEXT("%Y-%m-%d_%H-%M-%S"));
    FileName = LogDirectory + "/" + UserID + "_" + ScN + "_" + Displ + "_" + Timestamp + ".csv";

    // Ensure the file exists, or create it with a header row
    if (!PlatformFile.FileExists(*FileName))
    {
        bool bFileCreated = FFileHelper::SaveStringToFile(TEXT("Timestamp, VX, VY, VZ, VRX,VRY,VRZ,TH,ST,BR,LHX,LHY,LHZ,RHX,RHY,RHZ,HX,HY,HZ,HRX,HRY,HRZ\n"), *FileName);

        if (!bFileCreated)
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to create log file: %s"), *FileName);
            return;
        }
        else
        {
            UE_LOG(LogTemp, Log, TEXT("Log file created successfully: %s"), *FileName);
        }
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("Log file already exists: %s"), *FileName);
    }

    // Start periodic writing (every 0.1s)
    if (GWorld)
    {
        GWorld->GetTimerManager().SetTimer(WriteTimerHandle, this, &UGameLogger::WriteToFile, 0.1f, true);
    }

    UE_LOG(LogTemp, Log, TEXT("Logger initialized successfully. Directory: %s, File: %s"), *LogDirectory, *FileName);
}



void UGameLogger::LogData(const FString& Data)
{
    FScopeLock Lock(&Mutex);  // Prevents race conditions
    LogBuffer.Add(Data);
}

void UGameLogger::WriteToFile()
{
    TArray<FString> TempBuffer;
    {
        FScopeLock Lock(&Mutex);
        TempBuffer = MoveTemp(LogBuffer);  // Move data to avoid extra copying
    }

    if (TempBuffer.Num() > 0)
    {
        FFileHelper::SaveStringArrayToFile(TempBuffer, *FileName, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_Append);
    }
}

void UGameLogger::SetLogFolder(const FString& FolderPath)
{
    IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

    // Ensure new directory exists
    if (!PlatformFile.DirectoryExists(*FolderPath))
    {
        PlatformFile.CreateDirectoryTree(*FolderPath);
    }

    LogDirectory = FolderPath;
    FileName = LogDirectory + "/" + FPaths::GetCleanFilename(FileName);
}

void UGameLogger::RenameLogFile(const FString& NewFileName)
{
    FString NewFilePath = LogDirectory + "/" + NewFileName + ".csv";

    IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
    if (PlatformFile.FileExists(*FileName))
    {
        PlatformFile.MoveFile(*NewFilePath, *FileName);
    }

    FileName = NewFilePath;
}
