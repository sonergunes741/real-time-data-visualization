/*
 * STM32 Real-Time Sensor Monitor
 * Windows Classic Theme - Expert-Oriented Utility UI
 * 
 * Compile: gcc -Wall -O2 -o sensor_monitor.exe sensor_monitor.c -lgdi32 -lcomctl32 -mwindows
 */

#define _WIN32_WINNT 0x0501
#define WINVER 0x0501

#include <windows.h>
#include <commctrl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// ============================================================================
// DEFINITIONS
// ============================================================================

#define MAX_DATA_POINTS 300
#define GRAPH_UPDATE_MS 100
#define MAX_LOG_LINES 100

// Control IDs
#define IDC_COMBO_PORT      101
#define IDC_COMBO_BAUD      102
#define IDC_BTN_CONNECT     103
#define IDC_BTN_DISCONNECT  104
#define IDC_RADIO_FREERTOS  105
#define IDC_RADIO_BAREMETAL 106
#define IDC_BTN_RESET_STATS 107
#define IDC_TIMER_GRAPH     108
#define IDC_STATIC_STATUS   109
#define IDC_CHECK_LOG       110
#define IDC_EDIT_RANGE      111
#define IDC_BTN_APPLY_RANGE 112
#define IDC_COMBO_BUSYWAIT  113
#define IDC_BTN_ACK_ALERT   114
#define IDC_EDIT_THRESHOLD  115
#define IDC_BTN_SET_THRESH  116

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

HWND hMainWnd;
HWND hComboPort, hComboBaud;
HWND hBtnConnect, hBtnDisconnect;
HWND hRadioFreeRTOS, hRadioBareMetal;
HWND hEditRange;
HWND hBtnApplyRange;
HWND hComboBusyWait;
HWND hBtnResetStats;
HWND hCheckLog;
HWND hStatusBar;
// Alert section
HWND hAlertStatus;
HWND hBtnAckAlert;
HWND hEditThreshold;
HWND hBtnSetThresh;

HANDLE hSerial = INVALID_HANDLE_VALUE;
HANDLE hReadThread = NULL;
volatile BOOL bRunning = FALSE;

// Data storage
typedef struct {
    uint32_t timestamp;
    uint32_t distance;
    uint32_t latency;
    char mode;
} SensorData;

SensorData dataBuffer[MAX_DATA_POINTS];
int dataCount = 0;
int dataHead = 0;
CRITICAL_SECTION csData;

// Current values
char currentMode = '?';
uint32_t currentDistance = 0;

// Distance averaging
uint32_t distanceSum = 0;
uint32_t distanceCount = 0;
uint32_t lastValidDistance = 0;

// Busy-Wait simulation (0.5s delay buffer)
BOOL busyWaitEnabled = FALSE;
#define DELAY_BUFFER_SIZE 10  // 0.5 sec / 50ms = 10 samples
SensorData delayBuffer[DELAY_BUFFER_SIZE];
int delayBufferHead = 0;
int delayBufferCount = 0;

// Latency statistics - her mod için ayrı
uint32_t freertosLatencySum = 0;
uint32_t freertosLatencyCount = 0;
uint32_t freertosLatencyMin = 0xFFFFFFFF;
uint32_t freertosLatencyMax = 0;

uint32_t baremetalLatencySum = 0;
uint32_t baremetalLatencyCount = 0;
uint32_t baremetalLatencyMin = 0xFFFFFFFF;
uint32_t baremetalLatencyMax = 0;

// Graph range (cm)
int graphMaxRange = 20;

// Connected mode
char connectedMode = '?';

// Log settings
BOOL logEnabled = FALSE;
FILE* logFile = NULL;
int logLineCount = 0;

// Alert state (from STM32)
BOOL alertActive = FALSE;
uint32_t alertSamples = 0;
uint32_t alertDuration = 0;
int alertThreshold = 0;

// ============================================================================
// LOG FILE FUNCTIONS
// ============================================================================

void InitLogFile() {
    logFile = fopen("sensor_log.txt", "w");
    if (logFile) {
        SYSTEMTIME st;
        GetLocalTime(&st);
        fprintf(logFile, "=== STM32 Sensor Monitor Log ===\n");
        fprintf(logFile, "Started: %04d-%02d-%02d %02d:%02d:%02d\n\n",
                st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
        fflush(logFile);
        logLineCount = 3;
    }
}

void WriteLog(const char* prefix, const char* msg) {
    if (!logFile || logLineCount >= MAX_LOG_LINES) return;
    
    SYSTEMTIME st;
    GetLocalTime(&st);
    fprintf(logFile, "[%02d:%02d:%02d.%03d] %s: %s\n",
            st.wHour, st.wMinute, st.wSecond, st.wMilliseconds, prefix, msg);
    fflush(logFile);
    logLineCount++;
}

void CloseLogFile() {
    if (logFile) {
        fprintf(logFile, "\n=== Log End ===\n");
        fclose(logFile);
        logFile = NULL;
    }
}

// ============================================================================
// SERIAL PORT FUNCTIONS
// ============================================================================

BOOL OpenSerialPort(const char* portName, int baudRate) {
    char fullPath[32];
    sprintf(fullPath, "\\\\.\\%s", portName);
    
    hSerial = CreateFileA(fullPath, GENERIC_READ | GENERIC_WRITE,
                          0, NULL, OPEN_EXISTING, 0, NULL);
    
    if (hSerial == INVALID_HANDLE_VALUE) {
        WriteLog("ERROR", "Failed to open COM port");
        return FALSE;
    }
    
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    
    if (!GetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return FALSE;
    }
    
    dcb.BaudRate = baudRate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    
    if (!SetCommState(hSerial, &dcb)) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return FALSE;
    }
    
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);
    
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    
    char msg[64];
    sprintf(msg, "Connected to %s @ %d", portName, baudRate);
    WriteLog("INFO", msg);
    
    return TRUE;
}

void CloseSerialPort() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        WriteLog("INFO", "Disconnected");
    }
}

BOOL SendSerialData(const char* data, int len) {
    if (hSerial == INVALID_HANDLE_VALUE) return FALSE;
    DWORD written;
    BOOL result = WriteFile(hSerial, data, len, &written, NULL);
    if (result) {
        char msg[64];
        sprintf(msg, "Sent: 0x%02X", (unsigned char)data[0]);
        WriteLog("TX", msg);
    }
    return result;
}

// ============================================================================
// DATA PARSING
// ============================================================================

void ParseSensorData(const char* line) {
    WriteLog("RX", line);
    
    if (strstr(line, "ACK:") != NULL) return;
    
    if (strstr(line, "STM32_READY:") != NULL) {
        char* modePtr = strstr(line, "MODE_");
        if (modePtr) currentMode = modePtr[5];
        return;
    }
    
    if (strstr(line, "BARE_METAL_MODE_STARTED") != NULL) {
        currentMode = 'B';
        return;
    }
    
    // Parse ALERT messages from STM32
    if (strstr(line, "ALERT:SAMPLES:") != NULL) {
        alertActive = TRUE;
        char* samplesPtr = strstr(line, "SAMPLES:");
        char* durationPtr = strstr(line, "DURATION:");
        char* threshPtr = strstr(line, "THRESHOLD:");
        
        if (samplesPtr) alertSamples = atol(samplesPtr + 8);
        if (durationPtr) alertDuration = atol(durationPtr + 9);
        if (threshPtr) alertThreshold = atoi(threshPtr + 10);
        return;
    }
    
    if (strstr(line, "ALERT:CLEARED") != NULL) {
        alertActive = FALSE;
        alertSamples = 0;
        alertDuration = 0;
        return;
    }
    
    SensorData data = {0};
    
    char* modePtr = strstr(line, "MODE:");
    char* timePtr = strstr(line, "TIME:");
    char* distPtr = strstr(line, "DIST:");
    char* latPtr = strstr(line, "LAT:");
    
    if (modePtr && timePtr && distPtr && latPtr) {
        data.mode = modePtr[5];
        data.timestamp = atol(timePtr + 5);
        data.distance = atol(distPtr + 5);
        data.latency = atol(latPtr + 4);
        
        currentMode = data.mode;
        currentDistance = data.distance;
        
        // Update distance stats
        distanceSum += data.distance;
        distanceCount++;
        lastValidDistance = data.distance;
        
        EnterCriticalSection(&csData);
        
        if (busyWaitEnabled) {
            // Buffer data with 2s delay
            delayBuffer[delayBufferHead] = data;
            delayBufferHead = (delayBufferHead + 1) % DELAY_BUFFER_SIZE;
            if (delayBufferCount < DELAY_BUFFER_SIZE) delayBufferCount++;
            
            // Push oldest data to graph when buffer is full
            if (delayBufferCount == DELAY_BUFFER_SIZE) {
                int oldestIdx = (delayBufferHead - delayBufferCount + DELAY_BUFFER_SIZE) % DELAY_BUFFER_SIZE;
                SensorData delayedData = delayBuffer[oldestIdx];
                
                dataBuffer[dataHead] = delayedData;
                dataHead = (dataHead + 1) % MAX_DATA_POINTS;
                if (dataCount < MAX_DATA_POINTS) dataCount++;
            }
        } else {
            // Normal mode - immediate display
            dataBuffer[dataHead] = data;
            dataHead = (dataHead + 1) % MAX_DATA_POINTS;
            if (dataCount < MAX_DATA_POINTS) dataCount++;
        }
        
        // Update latency stats for connected mode only
        if (data.mode == connectedMode) {
            if (data.mode == 'F') {
                freertosLatencySum += data.latency;
                freertosLatencyCount++;
                if (data.latency < freertosLatencyMin) freertosLatencyMin = data.latency;
                if (data.latency > freertosLatencyMax) freertosLatencyMax = data.latency;
            } else if (data.mode == 'B') {
                baremetalLatencySum += data.latency;
                baremetalLatencyCount++;
                if (data.latency < baremetalLatencyMin) baremetalLatencyMin = data.latency;
                if (data.latency > baremetalLatencyMax) baremetalLatencyMax = data.latency;
            }
        }
        
        LeaveCriticalSection(&csData);
    }
}

// ============================================================================
// SERIAL READ THREAD
// ============================================================================

DWORD WINAPI SerialReadThread(LPVOID lpParam) {
    char buffer[256];
    char lineBuffer[256];
    int linePos = 0;
    DWORD bytesRead;
    
    WriteLog("INFO", "Read thread started");
    
    while (bRunning) {
        if (ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL)) {
            for (DWORD i = 0; i < bytesRead; i++) {
                char c = buffer[i];
                if (c == '\n' || c == '\r') {
                    if (linePos > 0) {
                        lineBuffer[linePos] = '\0';
                        ParseSensorData(lineBuffer);
                        linePos = 0;
                    }
                } else if (linePos < sizeof(lineBuffer) - 1) {
                    lineBuffer[linePos++] = c;
                }
            }
        }
        Sleep(10);
    }
    
    return 0;
}

// ============================================================================
// GRAPH DRAWING
// ============================================================================

void DrawDistanceGraph(HDC hdc, RECT* rect) {
    HBRUSH hBrushBg = CreateSolidBrush(RGB(255, 255, 255));
    FillRect(hdc, rect, hBrushBg);
    DeleteObject(hBrushBg);
    
    HPEN hPenBorder = CreatePen(PS_SOLID, 2, RGB(100, 100, 100));
    SelectObject(hdc, hPenBorder);
    Rectangle(hdc, rect->left, rect->top, rect->right, rect->bottom);
    DeleteObject(hPenBorder);
    
    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(0, 0, 0));  // Siyah
    HFONT hFontTitle = CreateFontA(18, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        DEFAULT_QUALITY, DEFAULT_PITCH, "Segoe UI");
    SelectObject(hdc, hFontTitle);
    TextOutA(hdc, rect->left + 10, rect->top + 5, "Distance vs Time", 16);
    DeleteObject(hFontTitle);
    
    int graphLeft = rect->left + 55;
    int graphTop = rect->top + 30;
    int graphRight = rect->right - 20;
    int graphBottom = rect->bottom - 30;
    int graphWidth = graphRight - graphLeft;
    int graphHeight = graphBottom - graphTop;
    
    HPEN hPenGrid = CreatePen(PS_DOT, 1, RGB(220, 220, 220));
    SelectObject(hdc, hPenGrid);
    for (int i = 0; i <= 5; i++) {
        int y = graphTop + (graphHeight * i) / 5;
        MoveToEx(hdc, graphLeft, y, NULL);
        LineTo(hdc, graphRight, y);
    }
    DeleteObject(hPenGrid);
    
    HFONT hFontAxis = CreateFontA(14, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        DEFAULT_QUALITY, DEFAULT_PITCH, "Segoe UI");
    SelectObject(hdc, hFontAxis);
    SetTextColor(hdc, RGB(80, 80, 80));
    
    char label[16];
    sprintf(label, "%d cm", graphMaxRange);
    TextOutA(hdc, rect->left + 5, graphTop - 5, label, strlen(label));
    sprintf(label, "%d cm", graphMaxRange / 2);
    TextOutA(hdc, rect->left + 5, graphTop + graphHeight/2 - 5, label, strlen(label));
    sprintf(label, "0 cm");
    TextOutA(hdc, rect->left + 5, graphBottom - 10, label, strlen(label));
    DeleteObject(hFontAxis);
    
    EnterCriticalSection(&csData);
    
    if (dataCount > 1) {
        HPEN hPenData = CreatePen(PS_SOLID, 2, RGB(0, 0, 0));  // Siyah çizgi
        SelectObject(hdc, hPenData);
        
        int startIdx = (dataHead - dataCount + MAX_DATA_POINTS) % MAX_DATA_POINTS;
        BOOL firstPoint = TRUE;
        
        for (int i = 0; i < dataCount; i++) {
            int idx = (startIdx + i) % MAX_DATA_POINTS;
            int x = graphLeft + (graphWidth * i) / MAX_DATA_POINTS;
            
            int dist = dataBuffer[idx].distance;
            if (dist > graphMaxRange) dist = graphMaxRange;
            int y = graphBottom - (graphHeight * dist) / graphMaxRange;
            
            if (firstPoint) {
                MoveToEx(hdc, x, y, NULL);
                firstPoint = FALSE;
            } else {
                LineTo(hdc, x, y);
            }
        }
        
        DeleteObject(hPenData);
    }
    
    LeaveCriticalSection(&csData);
}

void DrawLatencyBoxes(HDC hdc, RECT* rect) {
    SetBkMode(hdc, TRANSPARENT);
    
    EnterCriticalSection(&csData);
    float freertosAvg = freertosLatencyCount > 0 ? 
        (float)freertosLatencySum / freertosLatencyCount : 0;
    float baremetalAvg = baremetalLatencyCount > 0 ? 
        (float)baremetalLatencySum / baremetalLatencyCount : 0;
    float ratio = (baremetalAvg > 0 && freertosAvg > 0) ? freertosAvg / baremetalAvg : 0;
    
    // İstatistikleri kopyala
    uint32_t fCount = freertosLatencyCount;
    uint32_t fMin = freertosLatencyMin;
    uint32_t fMax = freertosLatencyMax;
    uint32_t bCount = baremetalLatencyCount;
    uint32_t bMin = baremetalLatencyMin;
    uint32_t bMax = baremetalLatencyMax;
    LeaveCriticalSection(&csData);
    
    int totalWidth = rect->right - rect->left;
    int boxWidth = totalWidth / 3;  // 3 kutu
    int boxHeight = rect->bottom - rect->top;
    int halfHeight = boxHeight / 2;
    
    HFONT hFontTitle = CreateFontA(16, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        DEFAULT_QUALITY, DEFAULT_PITCH, "Segoe UI");
    HFONT hFontValue = CreateFontA(28, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        DEFAULT_QUALITY, DEFAULT_PITCH, "Segoe UI");
    HFONT hFontStats = CreateFontA(14, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        DEFAULT_QUALITY, DEFAULT_PITCH, "Segoe UI");
    
    char val[64];
    
    // ========== Box 1: FreeRTOS ==========
    RECT box1 = {rect->left, rect->top, rect->left + boxWidth, rect->bottom};
    HBRUSH hBrush1 = CreateSolidBrush(RGB(255, 255, 255));
    FillRect(hdc, &box1, hBrush1);
    DeleteObject(hBrush1);
    HPEN hPen1 = CreatePen(PS_SOLID, 1, RGB(128, 128, 128));
    SelectObject(hdc, hPen1);
    SelectObject(hdc, GetStockObject(NULL_BRUSH));
    Rectangle(hdc, box1.left, box1.top, box1.right, box1.bottom);
    DeleteObject(hPen1);
    
    // Başlık
    SelectObject(hdc, hFontTitle);
    SetTextColor(hdc, RGB(0, 0, 0));
    TextOutA(hdc, rect->left + 10, rect->top + 8, "FreeRTOS", 8);
    
    // Avg değeri
    if (fCount > 0) {
        sprintf(val, "%.1f ms", freertosAvg);
    } else {
        sprintf(val, "N/A");
    }
    SelectObject(hdc, hFontValue);
    TextOutA(hdc, rect->left + 10, rect->top + 32, val, strlen(val));
    
    // İstatistikler
    SelectObject(hdc, hFontStats);
    SetTextColor(hdc, RGB(80, 80, 80));
    sprintf(val, "Samples: %lu", (unsigned long)fCount);
    TextOutA(hdc, rect->left + 10, rect->top + 80, val, strlen(val));
    
    if (fCount > 0) {
        sprintf(val, "Min: %lu ms", (unsigned long)fMin);
        TextOutA(hdc, rect->left + 10, rect->top + 102, val, strlen(val));
        sprintf(val, "Max: %lu ms", (unsigned long)fMax);
        TextOutA(hdc, rect->left + 10, rect->top + 124, val, strlen(val));
        sprintf(val, "Avg: %.1f ms", freertosAvg);
        TextOutA(hdc, rect->left + 10, rect->top + 146, val, strlen(val));
    } else {
        TextOutA(hdc, rect->left + 10, rect->top + 102, "Min: --", 7);
        TextOutA(hdc, rect->left + 10, rect->top + 124, "Max: --", 7);
        TextOutA(hdc, rect->left + 10, rect->top + 146, "Avg: --", 7);
    }
    
    // ========== Box 2: BareMetal ==========
    int box2X = rect->left + boxWidth;
    RECT box2 = {box2X, rect->top, box2X + boxWidth, rect->bottom};
    HBRUSH hBrush2 = CreateSolidBrush(RGB(255, 255, 255));
    FillRect(hdc, &box2, hBrush2);
    DeleteObject(hBrush2);
    HPEN hPen2 = CreatePen(PS_SOLID, 1, RGB(128, 128, 128));
    SelectObject(hdc, hPen2);
    SelectObject(hdc, GetStockObject(NULL_BRUSH));
    Rectangle(hdc, box2.left, box2.top, box2.right, box2.bottom);
    DeleteObject(hPen2);
    
    // Başlık
    SelectObject(hdc, hFontTitle);
    SetTextColor(hdc, RGB(0, 0, 0));
    TextOutA(hdc, box2X + 10, rect->top + 8, "BareMetal", 9);
    
    // Avg değeri
    if (bCount > 0) {
        sprintf(val, "%.1f ms", baremetalAvg);
    } else {
        sprintf(val, "N/A");
    }
    SelectObject(hdc, hFontValue);
    TextOutA(hdc, box2X + 10, rect->top + 32, val, strlen(val));
    
    // İstatistikler
    SelectObject(hdc, hFontStats);
    SetTextColor(hdc, RGB(80, 80, 80));
    sprintf(val, "Samples: %lu", (unsigned long)bCount);
    TextOutA(hdc, box2X + 10, rect->top + 80, val, strlen(val));
    
    if (bCount > 0) {
        sprintf(val, "Min: %lu ms", (unsigned long)bMin);
        TextOutA(hdc, box2X + 10, rect->top + 102, val, strlen(val));
        sprintf(val, "Max: %lu ms", (unsigned long)bMax);
        TextOutA(hdc, box2X + 10, rect->top + 124, val, strlen(val));
        sprintf(val, "Avg: %.1f ms", baremetalAvg);
        TextOutA(hdc, box2X + 10, rect->top + 146, val, strlen(val));
    } else {
        TextOutA(hdc, box2X + 10, rect->top + 102, "Min: --", 7);
        TextOutA(hdc, box2X + 10, rect->top + 124, "Max: --", 7);
        TextOutA(hdc, box2X + 10, rect->top + 146, "Avg: --", 7);
    }
    
    // ========== Box 3: Ratio + Distance (alt alta) ==========
    int box3X = rect->left + 2 * boxWidth;
    
    // Üst yarı: Ratio
    RECT box3Top = {box3X, rect->top, rect->right, rect->top + halfHeight};
    HBRUSH hBrush3 = CreateSolidBrush(RGB(255, 255, 255));
    FillRect(hdc, &box3Top, hBrush3);
    DeleteObject(hBrush3);
    HPEN hPen3 = CreatePen(PS_SOLID, 1, RGB(128, 128, 128));
    SelectObject(hdc, hPen3);
    SelectObject(hdc, GetStockObject(NULL_BRUSH));
    Rectangle(hdc, box3Top.left, box3Top.top, box3Top.right, box3Top.bottom);
    DeleteObject(hPen3);
    
    // Ratio başlık
    SelectObject(hdc, hFontTitle);
    SetTextColor(hdc, RGB(0, 0, 0));
    TextOutA(hdc, box3X + 10, rect->top + 8, "Ratio", 5);
    
    // Ratio değeri
    if (ratio > 0) {
        sprintf(val, "%.2fx", ratio);
    } else {
        sprintf(val, "N/A");
    }
    SelectObject(hdc, hFontValue);
    TextOutA(hdc, box3X + 10, rect->top + 32, val, strlen(val));
    
    // Alt yarı: Distance
    RECT box3Bottom = {box3X, rect->top + halfHeight, rect->right, rect->bottom};
    HBRUSH hBrush4 = CreateSolidBrush(RGB(255, 255, 255));
    FillRect(hdc, &box3Bottom, hBrush4);
    DeleteObject(hBrush4);
    HPEN hPen4 = CreatePen(PS_SOLID, 1, RGB(128, 128, 128));
    SelectObject(hdc, hPen4);
    SelectObject(hdc, GetStockObject(NULL_BRUSH));
    Rectangle(hdc, box3Bottom.left, box3Bottom.top, box3Bottom.right, box3Bottom.bottom);
    DeleteObject(hPen4);
    
    // Distance başlık
    SelectObject(hdc, hFontTitle);
    SetTextColor(hdc, RGB(0, 0, 0));
    TextOutA(hdc, box3X + 10, rect->top + halfHeight + 8, "Distance", 8);
    
    // Distance değeri
    sprintf(val, "%lu cm", (unsigned long)currentDistance);
    SelectObject(hdc, hFontValue);
    TextOutA(hdc, box3X + 10, rect->top + halfHeight + 32, val, strlen(val));
    
    DeleteObject(hFontTitle);
    DeleteObject(hFontValue);
    DeleteObject(hFontStats);
}



// ============================================================================
// WINDOW PROCEDURE
// ============================================================================

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {
    switch (message) {
        case WM_CREATE: {
            InitializeCriticalSection(&csData);
            InitLogFile();
            
            // ===== PORT SETUP SECTION ===== (y: 10-130, height: 120)
            CreateWindowA("BUTTON", "Port Setup", 
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                10, 10, 220, 120, hWnd, NULL, NULL, NULL);
            
            CreateWindowA("STATIC", "Port",
                WS_CHILD | WS_VISIBLE, 25, 35, 40, 20, hWnd, NULL, NULL, NULL);
            
            hComboPort = CreateWindowA("COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
                70, 32, 140, 200, hWnd, (HMENU)IDC_COMBO_PORT, NULL, NULL);
            
            for (int i = 1; i <= 20; i++) {
                char port[16];
                sprintf(port, "COM%d", i);
                SendMessageA(hComboPort, CB_ADDSTRING, 0, (LPARAM)port);
            }
            SendMessageA(hComboPort, CB_SETCURSEL, 3, 0);
            
            CreateWindowA("STATIC", "Baud",
                WS_CHILD | WS_VISIBLE, 25, 65, 40, 20, hWnd, NULL, NULL, NULL);
            
            hComboBaud = CreateWindowA("COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
                70, 62, 140, 200, hWnd, (HMENU)IDC_COMBO_BAUD, NULL, NULL);
            
            SendMessageA(hComboBaud, CB_ADDSTRING, 0, (LPARAM)"9600");
            SendMessageA(hComboBaud, CB_ADDSTRING, 0, (LPARAM)"115200");
            SendMessageA(hComboBaud, CB_SETCURSEL, 1, 0);
            
            // Butonlar aşağıda, daha fazla boşluk - Connect başlangıçta inactive
            hBtnConnect = CreateWindowA("BUTTON", "Connect",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_DISABLED,
                25, 98, 90, 26, hWnd, (HMENU)IDC_BTN_CONNECT, NULL, NULL);
            
            hBtnDisconnect = CreateWindowA("BUTTON", "Disconnect",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_DISABLED,
                120, 98, 90, 26, hWnd, (HMENU)IDC_BTN_DISCONNECT, NULL, NULL);
            
            // ===== RUN MODE SECTION ===== (y: 135-195, height: 60 - Apply None)
            CreateWindowA("BUTTON", "Run Mode", 
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                10, 135, 220, 60, hWnd, NULL, NULL, NULL);
            
            // BareMetal solda, FreeRTOS sağda
            hRadioBareMetal = CreateWindowA("BUTTON", "BareMetal",
                WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_GROUP,
                25, 160, 90, 20, hWnd, (HMENU)IDC_RADIO_BAREMETAL, NULL, NULL);
            
            hRadioFreeRTOS = CreateWindowA("BUTTON", "FreeRTOS",
                WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON,
                120, 160, 90, 20, hWnd, (HMENU)IDC_RADIO_FREERTOS, NULL, NULL);
            
            // ===== GRAPH RANGE SECTION ===== (y: 200-260)
            CreateWindowA("BUTTON", "Graph Range", 
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                10, 200, 220, 55, hWnd, NULL, NULL, NULL);
            
            // Range label
            CreateWindowA("STATIC", "0 -",
                WS_CHILD | WS_VISIBLE, 25, 223, 25, 16, hWnd, NULL, NULL, NULL);
            
            // Range text input
            hEditRange = CreateWindowA("EDIT", "20",
                WS_CHILD | WS_VISIBLE | WS_BORDER | ES_NUMBER,
                52, 220, 45, 22, hWnd, (HMENU)IDC_EDIT_RANGE, NULL, NULL);
            
            // cm label
            CreateWindowA("STATIC", "cm",
                WS_CHILD | WS_VISIBLE, 100, 223, 25, 16, hWnd, NULL, NULL, NULL);
            
            // Apply butonu
            hBtnApplyRange = CreateWindowA("BUTTON", "Apply",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                135, 219, 55, 24, hWnd, (HMENU)IDC_BTN_APPLY_RANGE, NULL, NULL);
            
            // ===== SCHEDULING OVERHEAD SECTION =====
            CreateWindowA("BUTTON", "Scheduling Overhead", 
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                10, 260, 220, 55, hWnd, NULL, NULL, NULL);
            
            CreateWindowA("STATIC", "Busy-Wait:",
                WS_CHILD | WS_VISIBLE, 25, 283, 70, 16, hWnd, NULL, NULL, NULL);
            
            hComboBusyWait = CreateWindowA("COMBOBOX", "",
                WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
                100, 280, 80, 100, hWnd, (HMENU)IDC_COMBO_BUSYWAIT, NULL, NULL);
            
            SendMessageA(hComboBusyWait, CB_ADDSTRING, 0, (LPARAM)"Inactive");
            SendMessageA(hComboBusyWait, CB_ADDSTRING, 0, (LPARAM)"Active");
            SendMessageA(hComboBusyWait, CB_SETCURSEL, 0, 0);
            
            // ===== CRITICAL ALERT SECTION =====
            CreateWindowA("BUTTON", "Critical Alert", 
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                10, 320, 220, 115, hWnd, NULL, NULL, NULL);
            
            // Threshold input
            CreateWindowA("STATIC", "Threshold:",
                WS_CHILD | WS_VISIBLE, 25, 343, 65, 16, hWnd, NULL, NULL, NULL);
            
            hEditThreshold = CreateWindowA("EDIT", "10",
                WS_CHILD | WS_VISIBLE | WS_BORDER | ES_NUMBER,
                95, 340, 40, 20, hWnd, (HMENU)IDC_EDIT_THRESHOLD, NULL, NULL);
            
            CreateWindowA("STATIC", "cm",
                WS_CHILD | WS_VISIBLE, 140, 343, 25, 16, hWnd, NULL, NULL, NULL);
            
            hBtnSetThresh = CreateWindowA("BUTTON", "Set",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                170, 339, 45, 22, hWnd, (HMENU)IDC_BTN_SET_THRESH, NULL, NULL);
            
            // Alert status display
            hAlertStatus = CreateWindowA("STATIC", "Status: OK",
                WS_CHILD | WS_VISIBLE, 25, 365, 190, 32, hWnd, NULL, NULL, NULL);
            
            // Acknowledge button
            hBtnAckAlert = CreateWindowA("BUTTON", "Acknowledge",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_DISABLED,
                25, 400, 190, 24, hWnd, (HMENU)IDC_BTN_ACK_ALERT, NULL, NULL);
            
            // ===== OPTIONS SECTION =====
            CreateWindowA("BUTTON", "Options", 
                WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
                10, 440, 220, 75, hWnd, NULL, NULL, NULL);
            
            hCheckLog = CreateWindowA("BUTTON", "Enable Logging",
                WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
                25, 460, 150, 20, hWnd, (HMENU)IDC_CHECK_LOG, NULL, NULL);
            
            hBtnResetStats = CreateWindowA("BUTTON", "Reset Stats",
                WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                25, 485, 190, 24, hWnd, (HMENU)IDC_BTN_RESET_STATS, NULL, NULL);
            
            // ===== STATUS BAR =====
            hStatusBar = CreateWindowA("STATIC", "Select Run Mode",
                WS_CHILD | WS_VISIBLE | SS_SUNKEN,
                10, 520, 880, 22, hWnd, (HMENU)IDC_STATIC_STATUS, NULL, NULL);
            
            SetTimer(hWnd, IDC_TIMER_GRAPH, GRAPH_UPDATE_MS, NULL);
            
            // Disable FreeRTOS-only controls until mode is selected
            EnableWindow(hComboBusyWait, FALSE);
            EnableWindow(hEditThreshold, FALSE);
            EnableWindow(hBtnSetThresh, FALSE);
            
            WriteLog("INFO", "Application started");
            
            break;
        }
        
        case WM_COMMAND: {
            int wmId = LOWORD(wParam);
            
            switch (wmId) {
                case IDC_BTN_CONNECT: {
                    // Mod seçim kontrolü
                    BOOL freertosSelected = SendMessageA(hRadioFreeRTOS, BM_GETCHECK, 0, 0) == BST_CHECKED;
                    BOOL baremetalSelected = SendMessageA(hRadioBareMetal, BM_GETCHECK, 0, 0) == BST_CHECKED;
                    
                    if (!freertosSelected && !baremetalSelected) {
                        SetWindowTextA(hStatusBar, "Select Run Mode first!");
                        break;
                    }
                    
                    char port[16];
                    GetWindowTextA(hComboPort, port, sizeof(port));
                    char baudStr[16];
                    GetWindowTextA(hComboBaud, baudStr, sizeof(baudStr));
                    int baud = atoi(baudStr);
                    
                    if (OpenSerialPort(port, baud)) {
                        bRunning = TRUE;
                        hReadThread = CreateThread(NULL, 0, SerialReadThread, NULL, 0, NULL);
                        EnableWindow(hBtnConnect, FALSE);
                        EnableWindow(hBtnDisconnect, TRUE);
                        // Mod ve log seçeneğini devre dışı bırak
                        EnableWindow(hRadioFreeRTOS, FALSE);
                        EnableWindow(hRadioBareMetal, FALSE);
                        EnableWindow(hCheckLog, FALSE);
                        SetWindowTextA(hStatusBar, "Connected");
                        
                        Sleep(100);
                        
                        // Bağlı olunan modu ayarla
                        connectedMode = freertosSelected ? 'F' : 'B';
                        
                        // Reset stats for connected mode only
                        EnterCriticalSection(&csData);
                        if (connectedMode == 'F') {
                            freertosLatencySum = 0;
                            freertosLatencyCount = 0;
                            freertosLatencyMin = 0xFFFFFFFF;
                            freertosLatencyMax = 0;
                        } else {
                            baremetalLatencySum = 0;
                            baremetalLatencyCount = 0;
                            baremetalLatencyMin = 0xFFFFFFFF;
                            baremetalLatencyMax = 0;
                        }
                        dataCount = 0;
                        LeaveCriticalSection(&csData);
                        
                        // Enable FreeRTOS-only controls (Busy-Wait, Critical Alert)
                        BOOL isFreeRTOS = (connectedMode == 'F');
                        EnableWindow(hComboBusyWait, isFreeRTOS);
                        EnableWindow(hEditThreshold, isFreeRTOS);
                        EnableWindow(hBtnSetThresh, isFreeRTOS);
                        
                        // Send mode command
                        char cmd = connectedMode;
                        SendSerialData(&cmd, 1);
                    } else {
                        SetWindowTextA(hStatusBar, "Connection Failed");
                    }
                    break;
                }
                
                case IDC_BTN_DISCONNECT: {
                    bRunning = FALSE;
                    if (hReadThread) {
                        WaitForSingleObject(hReadThread, 1000);
                        CloseHandle(hReadThread);
                        hReadThread = NULL;
                    }
                    CloseSerialPort();
                    
                    connectedMode = '?';
                    
                    EnableWindow(hBtnConnect, TRUE);
                    EnableWindow(hBtnDisconnect, FALSE);
                    EnableWindow(hRadioFreeRTOS, TRUE);
                    EnableWindow(hRadioBareMetal, TRUE);
                    EnableWindow(hCheckLog, TRUE);
                    // Re-enable FreeRTOS controls
                    EnableWindow(hComboBusyWait, TRUE);
                    EnableWindow(hEditThreshold, TRUE);
                    EnableWindow(hBtnSetThresh, TRUE);
                    SetWindowTextA(hStatusBar, "Disconnected");
                    break;
                }
                
                // Radio button tıklandığında Connect'i activeleştir
                case IDC_RADIO_FREERTOS: {
                    EnableWindow(hBtnConnect, TRUE);
                    // Enable FreeRTOS-only sections
                    EnableWindow(hComboBusyWait, TRUE);
                    EnableWindow(hEditThreshold, TRUE);
                    EnableWindow(hBtnSetThresh, TRUE);
                    SetWindowTextA(hStatusBar, "FreeRTOS Mode Selected");
                    break;
                }
                
                case IDC_RADIO_BAREMETAL: {
                    EnableWindow(hBtnConnect, TRUE);
                    // Disable FreeRTOS-only sections
                    EnableWindow(hComboBusyWait, FALSE);
                    EnableWindow(hEditThreshold, FALSE);
                    EnableWindow(hBtnSetThresh, FALSE);
                    SetWindowTextA(hStatusBar, "BareMetal Mode Selected");
                    break;
                }
                
                case IDC_BTN_APPLY_RANGE: {
                    // Text input'tan değeri oku
                    char valueStr[16];
                    GetWindowTextA(hEditRange, valueStr, sizeof(valueStr));
                    int value = atoi(valueStr);
                    
                    // Minimum 5cm kontrolü
                    if (value < 5) {
                        SetWindowTextA(hStatusBar, "Minimum range is 5 cm");
                        SetWindowTextA(hEditRange, "5");
                        value = 5;
                    } else if (value > 1000) {
                        SetWindowTextA(hStatusBar, "Maximum range is 1000 cm");
                        SetWindowTextA(hEditRange, "1000");
                        value = 1000;
                    } else {
                        char status[64];
                        sprintf(status, "Graph range: 0 - %d cm", value);
                        SetWindowTextA(hStatusBar, status);
                    }
                    
                    graphMaxRange = value;
                    break;
                }
                
                case IDC_COMBO_BUSYWAIT: {
                    if (HIWORD(wParam) == CBN_SELCHANGE) {
                        int idx = SendMessageA(hComboBusyWait, CB_GETCURSEL, 0, 0);
                        
                        if (idx == 0) {
                            busyWaitEnabled = FALSE;
                            SetWindowTextA(hStatusBar, "Busy-Wait: Inactive");
                        } else {
                            busyWaitEnabled = TRUE;
                            delayBufferHead = 0;
                            delayBufferCount = 0;
                            SetWindowTextA(hStatusBar, "Busy-Wait: Active");
                        }
                    }
                    break;
                }
                
                case IDC_BTN_RESET_STATS: {
                    EnterCriticalSection(&csData);
                    // FreeRTOS stats
                    freertosLatencySum = 0;
                    freertosLatencyCount = 0;
                    freertosLatencyMin = 0xFFFFFFFF;
                    freertosLatencyMax = 0;
                    // BareMetal stats
                    baremetalLatencySum = 0;
                    baremetalLatencyCount = 0;
                    baremetalLatencyMin = 0xFFFFFFFF;
                    baremetalLatencyMax = 0;
                    // Graph data
                    dataCount = 0;
                    dataHead = 0;
                    LeaveCriticalSection(&csData);
                    SetWindowTextA(hStatusBar, "Statistics Reset");
                    WriteLog("INFO", "Statistics reset by user");
                    break;
                }
                
                case IDC_CHECK_LOG: {
                    logEnabled = (SendMessageA(hCheckLog, BM_GETCHECK, 0, 0) == BST_CHECKED);
                    if (logEnabled) {
                        SetWindowTextA(hStatusBar, "Logging Enabled");
                    } else {
                        SetWindowTextA(hStatusBar, "Logging Disabled");
                    }
                    break;
                }
                
                case IDC_BTN_ACK_ALERT: {
                    if (hSerial != INVALID_HANDLE_VALUE) {
                        char cmd = 'A';
                        DWORD written;
                        WriteFile(hSerial, &cmd, 1, &written, NULL);
                        SetWindowTextA(hStatusBar, "Alert Acknowledged");
                    }
                    break;
                }
                
                case IDC_BTN_SET_THRESH: {
                    if (hSerial != INVALID_HANDLE_VALUE) {
                        char text[16];
                        GetWindowTextA(hEditThreshold, text, 16);
                        int value = atoi(text);
                        
                        char cmd;
                        if (value <= 5) { cmd = 'T'; value = 5; }
                        else if (value <= 10) { cmd = 'U'; value = 10; }
                        else if (value <= 15) { cmd = 'V'; value = 15; }
                        else if (value <= 20) { cmd = 'W'; value = 20; }
                        else if (value <= 25) { cmd = 'X'; value = 25; }
                        else { cmd = 'Y'; value = 30; }
                        
                        DWORD written;
                        WriteFile(hSerial, &cmd, 1, &written, NULL);
                        
                        char status[64];
                        sprintf(status, "Threshold set: %d cm", value);
                        SetWindowTextA(hStatusBar, status);
                        
                        char valueText[8];
                        sprintf(valueText, "%d", value);
                        SetWindowTextA(hEditThreshold, valueText);
                    }
                    break;
                }
            }
            break;
        }
        
        case WM_TIMER: {
            if (wParam == IDC_TIMER_GRAPH) {
                // Update alert display
                if (alertActive) {
                    char alertText[128];
                    sprintf(alertText, "ALERT! Samples: %u\nDuration: %u ms", 
                            alertSamples, alertDuration);
                    SetWindowTextA(hAlertStatus, alertText);
                    EnableWindow(hBtnAckAlert, TRUE);
                } else {
                    SetWindowTextA(hAlertStatus, "Status: OK");
                    EnableWindow(hBtnAckAlert, FALSE);
                }
                
                // Update graphs
                RECT graphRect = {240, 10, 890, 320};
                InvalidateRect(hWnd, &graphRect, FALSE);
                RECT latencyRect = {240, 330, 890, 510};
                InvalidateRect(hWnd, &latencyRect, FALSE);
            }
            break;
        }
        
        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);
            
            RECT graphRect = {240, 10, 890, 320};
            DrawDistanceGraph(hdc, &graphRect);
            
            // 4 kutu yan yana: FreeRTOS, BareMetal, Ratio, Distance
            // Her mod için istatistikler gösterilir
            RECT latencyRect = {240, 330, 890, 510};
            DrawLatencyBoxes(hdc, &latencyRect);
            
            EndPaint(hWnd, &ps);
            break;
        }
        
        case WM_DESTROY: {
            bRunning = FALSE;
            if (hReadThread) {
                WaitForSingleObject(hReadThread, 1000);
                CloseHandle(hReadThread);
            }
            CloseSerialPort();
            CloseLogFile();
            DeleteCriticalSection(&csData);
            KillTimer(hWnd, IDC_TIMER_GRAPH);
            PostQuitMessage(0);
            break;
        }
        
        default:
            return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// ============================================================================
// MAIN
// ============================================================================

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine, int nCmdShow) {
    
    INITCOMMONCONTROLSEX icex;
    icex.dwSize = sizeof(INITCOMMONCONTROLSEX);
    icex.dwICC = ICC_WIN95_CLASSES;
    InitCommonControlsEx(&icex);
    
    WNDCLASSA wc = {0};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    wc.lpszClassName = "SensorMonitorClass";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    
    if (!RegisterClassA(&wc)) {
        MessageBoxA(NULL, "Window Registration Failed!", "Error", MB_ICONERROR);
        return 0;
    }
    
    hMainWnd = CreateWindowA(
        "SensorMonitorClass",
        "STM32 Real-Time Sensor Monitor",
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
        CW_USEDEFAULT, CW_USEDEFAULT,
        920, 580,
        NULL, NULL, hInstance, NULL
    );
    
    if (!hMainWnd) {
        MessageBoxA(NULL, "Window Creation Failed!", "Error", MB_ICONERROR);
        return 0;
    }
    
    ShowWindow(hMainWnd, nCmdShow);
    UpdateWindow(hMainWnd);
    
    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
    
    return (int)msg.wParam;
}
