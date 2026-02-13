#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "GameLogger.generated.h"

UCLASS(Blueprintable, BlueprintType)
class CARLAUNREAL_API UGameLogger : public UObject
{
    GENERATED_BODY()

private:
    FString LogDirectory;  // Folder where logs are stored
    FString FileName;      // The current log file name
    TArray<FString> LogBuffer;  // Buffer for storing log data before writing
    FCriticalSection Mutex;  // Ensures thread safety
    FTimerHandle WriteTimerHandle;  // Handles periodic writes

public:
    UGameLogger();

    /** Initializes the logger with a user ID and optional custom folder */
    UFUNCTION(BlueprintCallable, Category = "Logging")
    void InitializeLogger(const FString& UserID, const FString& ScN, const FString& Displ, const FString& CustomFolder = TEXT("Logs"));

    /** Logs a new data entry */
    UFUNCTION(BlueprintCallable, Category = "Logging")
    void LogData(const FString& Data);

    /** Writes buffered data to the file */
    UFUNCTION(BlueprintCallable, Category = "Logging")
    void WriteToFile();

    /** Sets a new log folder and creates it if it doesn't exist */
    UFUNCTION(BlueprintCallable, Category = "Logging")
    void SetLogFolder(const FString& FolderPath);

    /** Renames the log file */
    UFUNCTION(BlueprintCallable, Category = "Logging")
    void RenameLogFile(const FString& NewFileName);
};
