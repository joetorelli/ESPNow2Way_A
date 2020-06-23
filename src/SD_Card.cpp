#include "Arduino.h"
#include "SD_Card.h"
void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char *path)
{
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path))
    {
        Serial.println("Dir created");
    }
    else
    {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char *path)
{
    Serial.printf("Removing Dir: %s\n", path);
    if (fs.rmdir(path))
    {
        Serial.println("Dir removed");
    }
    else
    {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while (file.available())
    {
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("Message appended");
    }
    else
    {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2)
{
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2))
    {
        Serial.println("File renamed");
    }
    else
    {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char *path)
{
    Serial.printf("Deleting file: %s\n", path);
    if (fs.remove(path))
    {
        Serial.println("File deleted");
    }
    else
    {
        Serial.println("Delete failed");
    }
}

void Refresh_SD(BME_Sensor *SenVal)
{
}
//void Refresh_SD(DateTime *RTCClk, BME_Sensor *SenVal)
//{

//     String TimeStr = "";

//     if (RTCClk->year() < 10)
//     {
//         TimeStr = "0";
//     }
//     TimeStr = TimeStr + RTCClk->year();

//     TimeStr = TimeStr + "/";

//     if (RTCClk->month() < 10)
//     {
//         TimeStr = TimeStr + "0";
//     }
//     TimeStr = TimeStr + RTCClk->month();

//     TimeStr = TimeStr + "/";

//     if (RTCClk->day() < 10)
//     {
//         TimeStr = TimeStr + "0";
//     }
//     TimeStr = TimeStr + RTCClk->day();

//     TimeStr = TimeStr + " , ";

//     if (RTCClk->hour() < 10)
//     {
//         TimeStr = TimeStr + "0";
//     }
//     TimeStr = TimeStr + RTCClk->hour();

//     TimeStr = TimeStr + ":";

//     if (RTCClk->minute() < 10)
//     {
//         TimeStr = TimeStr + "0";
//     }
//     TimeStr = TimeStr + RTCClk->minute();

//     TimeStr = TimeStr + ":";

//     if (RTCClk->second() < 10)
//     {
//         TimeStr = TimeStr + "0";
//     }
//     TimeStr = TimeStr + RTCClk->second();

//     TimeStr = TimeStr + " T: " + SenVal->f_temperature + " H: " + SenVal->f_humidity;
//     DEBUGPRINT("*****************************************TimeStr= ");
//     DEBUGPRINTLN(TimeStr);

//     File myFile;
//     myFile = SD.open("/datalog.txt", FILE_APPEND);
//     if (myFile)
//     {
//         DEBUGPRINTLN("Writing Time");
//         myFile.println(TimeStr);
//         myFile.close();
//         DEBUGPRINTLN("Closed File");
//     }
//     else
//     {
//         Serial.println("File Error");
//     }
/*    
    // re-open the file for reading:
    myFile = SD.open("/datalog.txt");
    if (myFile)
    {
        DEBUG_PRINTLN("Read File");

        // read from the file until there's nothing else in it:
        while (myFile.available())
        {
            Serial.write(myFile.read());
        }
        // close the file:
        myFile.close();
    }
    else
    {
        // if the file didn't open, print an error:
        Serial.println("error opening");
    }
*/
//}

/* examples
    listDir(SD, "/", 0);
    createDir(SD, "/mydir");
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFileIO(SD, "/test.txt");
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
    */