/*
 * STM32 Real-Time Sensor Monitor
 * Linux Version - SDL2 UI with Double Buffer Mechanism
 * 
 * Architecture:
 *   Producer Thread: Reads serial data, writes to back buffer
 *   Consumer Thread: Main/render thread, reads from front buffer
 *   Double Buffer: Lock-free swap between producer and consumer
 * 
 * Compile: gcc -Wall -O2 -o sensor_monitor sensor_monitor_sdl.c `sdl2-config --cflags --libs` -lSDL2_ttf -lpthread -lm
 */

#define _GNU_SOURCE
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <time.h>
#include <dirent.h>
#include <math.h>

// ============================================================================
// DEFINITIONS
// ============================================================================

#define WINDOW_WIDTH  920
#define WINDOW_HEIGHT 580
#define MAX_DATA_POINTS 300
#define GRAPH_UPDATE_MS 50
#define MAX_LOG_LINES 100

// UI Layout
#define LEFT_PANEL_WIDTH 230
#define GRAPH_LEFT (LEFT_PANEL_WIDTH + 20)
#define GRAPH_TOP 30
#define GRAPH_WIDTH 640
#define GRAPH_HEIGHT 260

// Colors (RGBA)
#define COLOR_BG          0xF0F0F0FF
#define COLOR_WHITE       0xFFFFFFFF
#define COLOR_BLACK       0x000000FF
#define COLOR_GRAY        0x808080FF
#define COLOR_LIGHT_GRAY  0xD0D0D0FF
#define COLOR_DARK_GRAY   0x505050FF
#define COLOR_RED         0xFF0000FF
#define COLOR_GREEN       0x00AA00FF
#define COLOR_BLUE        0x0066CCFF

// ============================================================================
// DATA STRUCTURES
// ============================================================================

typedef struct {
    uint32_t timestamp;
    uint32_t distance;
    uint32_t latency;
    char mode;
} SensorData;

// Single buffer structure
typedef struct {
    SensorData data[MAX_DATA_POINTS];
    int count;
    int head;
    
    // Current values
    char currentMode;
    uint32_t currentDistance;
    
    // Latency stats - FreeRTOS
    uint32_t freertosLatencySum;
    uint32_t freertosLatencyCount;
    uint32_t freertosLatencyMin;
    uint32_t freertosLatencyMax;
    
    // Latency stats - BareMetal
    uint32_t baremetalLatencySum;
    uint32_t baremetalLatencyCount;
    uint32_t baremetalLatencyMin;
    uint32_t baremetalLatencyMax;
    
    // Alert state
    bool alertActive;
    uint32_t alertSamples;
    uint32_t alertDuration;
    int alertThreshold;
} DataBuffer;

// ============================================================================
// DOUBLE BUFFER MECHANISM
// ============================================================================

typedef struct {
    DataBuffer buffers[2];          // Two buffers for double buffering
    atomic_int writeIndex;          // Which buffer producer is writing to (0 or 1)
    atomic_bool swapReady;          // Flag indicating new data is ready
    pthread_mutex_t swapMutex;      // Mutex for swap operation
} DoubleBuffer;

static DoubleBuffer doubleBuffer;

// Initialize double buffer
void DoubleBuffer_Init(DoubleBuffer* db) {
    memset(&db->buffers[0], 0, sizeof(DataBuffer));
    memset(&db->buffers[1], 0, sizeof(DataBuffer));
    
    db->buffers[0].freertosLatencyMin = 0xFFFFFFFF;
    db->buffers[0].baremetalLatencyMin = 0xFFFFFFFF;
    db->buffers[1].freertosLatencyMin = 0xFFFFFFFF;
    db->buffers[1].baremetalLatencyMin = 0xFFFFFFFF;
    
    atomic_store(&db->writeIndex, 0);
    atomic_store(&db->swapReady, false);
    pthread_mutex_init(&db->swapMutex, NULL);
}

// Get write buffer (for producer)
DataBuffer* DoubleBuffer_GetWriteBuffer(DoubleBuffer* db) {
    int idx = atomic_load(&db->writeIndex);
    return &db->buffers[idx];
}

// Get read buffer (for consumer)
DataBuffer* DoubleBuffer_GetReadBuffer(DoubleBuffer* db) {
    int idx = atomic_load(&db->writeIndex);
    return &db->buffers[1 - idx];  // Opposite of write buffer
}

// Swap buffers (called by producer after writing)
void DoubleBuffer_Swap(DoubleBuffer* db) {
    pthread_mutex_lock(&db->swapMutex);
    
    int currentWrite = atomic_load(&db->writeIndex);
    int newWrite = 1 - currentWrite;
    
    // Copy current state to new write buffer before swap
    memcpy(&db->buffers[newWrite], &db->buffers[currentWrite], sizeof(DataBuffer));
    
    // Swap
    atomic_store(&db->writeIndex, newWrite);
    atomic_store(&db->swapReady, true);
    
    pthread_mutex_unlock(&db->swapMutex);
}

// Check if new data is ready (for consumer)
bool DoubleBuffer_HasNewData(DoubleBuffer* db) {
    return atomic_exchange(&db->swapReady, false);
}

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// SDL
SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;
TTF_Font* fontSmall = NULL;
TTF_Font* fontMedium = NULL;
TTF_Font* fontLarge = NULL;

// Serial
int serialFd = -1;
pthread_t producerThread;
volatile bool bRunning = false;

// UI State
char connectedMode = '?';
int graphMaxRange = 20;
bool logEnabled = false;

// UI Controls state
int selectedPort = 0;
int selectedBaud = 1;  // 115200
bool isConnected = false;
int selectedMode = -1;  // -1: none, 0: BareMetal, 1: FreeRTOS

// Text input state
char rangeInputText[16] = "20";
char thresholdInputText[16] = "";  // Empty by default - user must set it
int activeInputField = 0;  // 0=none, 1=range, 2=threshold
int inputCursorPos = 0;
int currentThreshold = 0;  // 0 means not set
bool thresholdEnabled = false;  // Only true after user clicks Set

// Button press state for visual feedback
Uint32 setButtonPressTime = 0;
bool setButtonPressed = false;

// Scheduling overhead buffer
#define OVERHEAD_BUFFER_SIZE 40
SensorData overheadBuffer[OVERHEAD_BUFFER_SIZE];
int overheadBufferHead = 0;
int overheadBufferCount = 0;
bool schedulingOverheadEnabled = false;
uint32_t displayedDistance = 0;

// Available ports
char availablePorts[20][32];
int portCount = 0;

// Log file
FILE* logFile = NULL;
int logLineCount = 0;

// Window dimensions (updated on resize)
int windowWidth = WINDOW_WIDTH;
int windowHeight = WINDOW_HEIGHT;

// Dynamic layout helper - returns current left panel width
int getLeftPanelWidth(void) {
    // 25% of window width, but minimum 230px, maximum 350px
    int w = windowWidth * 25 / 100;
    if (w < 230) w = 230;
    if (w > 350) w = 350;
    return w;
}

// Fixed height for bottom stat boxes (they grow less)
#define STAT_BOX_HEIGHT 180

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void get_timestamp_str(char *buffer, size_t size) {
    struct timeval tv;
    struct tm *tm_info;
    
    gettimeofday(&tv, NULL);
    tm_info = localtime(&tv.tv_sec);
    
    snprintf(buffer, size, "%02d:%02d:%02d.%03ld",
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             tv.tv_usec / 1000);
}

void get_date_str(char *buffer, size_t size) {
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    
    snprintf(buffer, size, "%04d-%02d-%02d %02d:%02d:%02d",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
}

// ============================================================================
// LOG FILE FUNCTIONS
// ============================================================================

void InitLogFile(void) {
    logFile = fopen("sensor_log.txt", "w");
    if (logFile) {
        char dateStr[64];
        get_date_str(dateStr, sizeof(dateStr));
        fprintf(logFile, "=== STM32 Sensor Monitor Log ===\n");
        fprintf(logFile, "Started: %s\n\n", dateStr);
        fflush(logFile);
        logLineCount = 3;
    }
}

void WriteLog(const char* prefix, const char* msg) {
    if (!logFile || !logEnabled || logLineCount >= MAX_LOG_LINES) return;
    
    char timestamp[32];
    get_timestamp_str(timestamp, sizeof(timestamp));
    fprintf(logFile, "[%s] %s: %s\n", timestamp, prefix, msg);
    fflush(logFile);
    logLineCount++;
}

void CloseLogFile(void) {
    if (logFile) {
        fprintf(logFile, "\n=== Log End ===\n");
        fclose(logFile);
        logFile = NULL;
    }
}

// ============================================================================
// SERIAL PORT FUNCTIONS
// ============================================================================

void EnumerateSerialPorts(void) {
    DIR *dir;
    struct dirent *entry;
    
    portCount = 0;
    
    dir = opendir("/dev");
    if (dir) {
        while ((entry = readdir(dir)) != NULL && portCount < 20) {
            if (strncmp(entry->d_name, "ttyUSB", 6) == 0 ||
                strncmp(entry->d_name, "ttyACM", 6) == 0) {
                strncpy(availablePorts[portCount], entry->d_name, 31);
                availablePorts[portCount][31] = '\0';
                portCount++;
            }
        }
        closedir(dir);
    }
    
    // If no ports found, add defaults
    if (portCount == 0) {
        strcpy(availablePorts[0], "ttyUSB0");
        strcpy(availablePorts[1], "ttyACM0");
        portCount = 2;
    }
}

int get_baud_constant(int baudRate) {
    switch (baudRate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return B115200;
    }
}

bool OpenSerialPort(const char* portName, int baudRate) {
    char fullPath[64];
    snprintf(fullPath, sizeof(fullPath), "/dev/%s", portName);
    
    serialFd = open(fullPath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if (serialFd < 0) {
        WriteLog("ERROR", "Failed to open serial port");
        return false;
    }
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serialFd, &tty) != 0) {
        close(serialFd);
        serialFd = -1;
        return false;
    }
    
    int baudConstant = get_baud_constant(baudRate);
    cfsetospeed(&tty, baudConstant);
    cfsetispeed(&tty, baudConstant);
    
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    
    if (tcsetattr(serialFd, TCSANOW, &tty) != 0) {
        close(serialFd);
        serialFd = -1;
        return false;
    }
    
    tcflush(serialFd, TCIOFLUSH);
    
    int status;
    ioctl(serialFd, TIOCMGET, &status);
    status |= TIOCM_DTR | TIOCM_RTS;
    ioctl(serialFd, TIOCMSET, &status);
    
    char msg[64];
    snprintf(msg, sizeof(msg), "Connected to %s @ %d", portName, baudRate);
    WriteLog("INFO", msg);
    
    return true;
}

void CloseSerialPort(void) {
    if (serialFd >= 0) {
        close(serialFd);
        serialFd = -1;
        WriteLog("INFO", "Disconnected");
    }
}

bool SendSerialData(const char* data, int len) {
    if (serialFd < 0) return false;
    ssize_t written = write(serialFd, data, len);
    if (written > 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Sent: 0x%02X", (unsigned char)data[0]);
        WriteLog("TX", msg);
    }
    return written == len;
}

// ============================================================================
// DATA PARSING (Producer writes to back buffer)
// ============================================================================

void ParseSensorData(const char* line) {
    WriteLog("RX", line);
    
    DataBuffer* wb = DoubleBuffer_GetWriteBuffer(&doubleBuffer);
    
    if (strstr(line, "ACK:") != NULL) return;
    
    if (strstr(line, "STM32_READY:") != NULL) {
        char* modePtr = strstr(line, "MODE_");
        if (modePtr) wb->currentMode = modePtr[5];
        DoubleBuffer_Swap(&doubleBuffer);
        return;
    }
    
    if (strstr(line, "BARE_METAL_MODE_STARTED") != NULL) {
        wb->currentMode = 'B';
        DoubleBuffer_Swap(&doubleBuffer);
        return;
    }
    
    // Parse ALERT messages
    if (strstr(line, "ALERT:SAMPLES:") != NULL) {
        wb->alertActive = true;
        char* samplesPtr = strstr(line, "SAMPLES:");
        char* durationPtr = strstr(line, "DURATION:");
        char* threshPtr = strstr(line, "THRESHOLD:");
        
        if (samplesPtr) wb->alertSamples = atol(samplesPtr + 8);
        if (durationPtr) wb->alertDuration = atol(durationPtr + 9);
        if (threshPtr) wb->alertThreshold = atoi(threshPtr + 10);
        DoubleBuffer_Swap(&doubleBuffer);
        return;
    }
    
    if (strstr(line, "ALERT:CLEARED") != NULL) {
        wb->alertActive = false;
        wb->alertSamples = 0;
        wb->alertDuration = 0;
        DoubleBuffer_Swap(&doubleBuffer);
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
        
        wb->currentMode = data.mode;
        
        // Scheduling overhead processing
        if (schedulingOverheadEnabled) {
            overheadBuffer[overheadBufferHead] = data;
            overheadBufferHead = (overheadBufferHead + 1) % OVERHEAD_BUFFER_SIZE;
            if (overheadBufferCount < OVERHEAD_BUFFER_SIZE) {
                overheadBufferCount++;
            }
            
            if (overheadBufferCount == OVERHEAD_BUFFER_SIZE) {
                int oldestIdx = (overheadBufferHead - overheadBufferCount + OVERHEAD_BUFFER_SIZE) % OVERHEAD_BUFFER_SIZE;
                data = overheadBuffer[oldestIdx];
                displayedDistance = data.distance;
            } else {
                displayedDistance = data.distance;
                return;
            }
        } else {
            displayedDistance = data.distance;
        }
        
        wb->currentDistance = displayedDistance;
        
        // Add to circular buffer
        wb->data[wb->head] = data;
        wb->head = (wb->head + 1) % MAX_DATA_POINTS;
        if (wb->count < MAX_DATA_POINTS) wb->count++;
        
        // Update latency stats for connected mode only
        if (data.mode == connectedMode) {
            if (data.mode == 'F') {
                wb->freertosLatencySum += data.latency;
                wb->freertosLatencyCount++;
                if (data.latency < wb->freertosLatencyMin) wb->freertosLatencyMin = data.latency;
                if (data.latency > wb->freertosLatencyMax) wb->freertosLatencyMax = data.latency;
            } else if (data.mode == 'B') {
                wb->baremetalLatencySum += data.latency;
                wb->baremetalLatencyCount++;
                if (data.latency < wb->baremetalLatencyMin) wb->baremetalLatencyMin = data.latency;
                if (data.latency > wb->baremetalLatencyMax) wb->baremetalLatencyMax = data.latency;
            }
        }
        
        // Swap buffers to make new data available to consumer
        DoubleBuffer_Swap(&doubleBuffer);
    }
}

// ============================================================================
// PRODUCER THREAD (Serial Reader)
// ============================================================================

void* ProducerThread(void* arg) {
    (void)arg;
    
    char buffer[256];
    char lineBuffer[256];
    int linePos = 0;
    
    WriteLog("INFO", "Producer thread started");
    
    while (bRunning) {
        ssize_t bytesRead = read(serialFd, buffer, sizeof(buffer) - 1);
        
        if (bytesRead > 0) {
            for (ssize_t i = 0; i < bytesRead; i++) {
                char c = buffer[i];
                if (c == '\n' || c == '\r') {
                    if (linePos > 0) {
                        lineBuffer[linePos] = '\0';
                        ParseSensorData(lineBuffer);
                        linePos = 0;
                    }
                } else if (linePos < (int)(sizeof(lineBuffer) - 1)) {
                    lineBuffer[linePos++] = c;
                }
            }
        }
        usleep(5000);  // 5ms sleep
    }
    
    WriteLog("INFO", "Producer thread stopped");
    return NULL;
}

// ============================================================================
// SDL HELPER FUNCTIONS
// ============================================================================

void SetColor(SDL_Renderer* r, uint32_t color) {
    SDL_SetRenderDrawColor(r, 
        (color >> 24) & 0xFF,
        (color >> 16) & 0xFF,
        (color >> 8) & 0xFF,
        color & 0xFF);
}

void DrawFilledRect(int x, int y, int w, int h, uint32_t color) {
    SetColor(renderer, color);
    SDL_Rect rect = {x, y, w, h};
    SDL_RenderFillRect(renderer, &rect);
}

void DrawRect(int x, int y, int w, int h, uint32_t color) {
    SetColor(renderer, color);
    SDL_Rect rect = {x, y, w, h};
    SDL_RenderDrawRect(renderer, &rect);
}

void DrawLine(int x1, int y1, int x2, int y2, uint32_t color) {
    SetColor(renderer, color);
    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
}

void DrawText(TTF_Font* font, const char* text, int x, int y, uint32_t color) {
    if (!font || !text || text[0] == '\0') return;
    
    SDL_Color sdlColor = {
        (color >> 24) & 0xFF,
        (color >> 16) & 0xFF,
        (color >> 8) & 0xFF,
        255
    };
    
    SDL_Surface* surface = TTF_RenderText_Blended(font, text, sdlColor);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        if (texture) {
            SDL_Rect dstRect = {x, y, surface->w, surface->h};
            SDL_RenderCopy(renderer, texture, NULL, &dstRect);
            SDL_DestroyTexture(texture);
        }
        SDL_FreeSurface(surface);
    }
}

void DrawButton(int x, int y, int w, int h, const char* text, bool enabled, bool pressed) {
    uint32_t bgColor = enabled ? (pressed ? 0xC0C0C0FF : 0xE0E0E0FF) : 0xD0D0D0FF;
    uint32_t textColor = enabled ? COLOR_BLACK : 0x808080FF;
    
    DrawFilledRect(x, y, w, h, bgColor);
    DrawRect(x, y, w, h, COLOR_GRAY);
    
    if (enabled && !pressed) {
        DrawLine(x, y, x + w - 1, y, COLOR_WHITE);
        DrawLine(x, y, x, y + h - 1, COLOR_WHITE);
    }
    
    // Center text
    int textW, textH;
    TTF_SizeText(fontSmall, text, &textW, &textH);
    DrawText(fontSmall, text, x + (w - textW) / 2, y + (h - textH) / 2, textColor);
}

void DrawCheckbox(int x, int y, const char* text, bool checked, bool enabled) {
    int boxSize = 16;
    uint32_t bgColor = enabled ? COLOR_WHITE : 0xE0E0E0FF;
    uint32_t textColor = enabled ? COLOR_BLACK : 0x808080FF;
    
    DrawFilledRect(x, y, boxSize, boxSize, bgColor);
    DrawRect(x, y, boxSize, boxSize, COLOR_GRAY);
    
    if (checked) {
        SetColor(renderer, COLOR_BLACK);
        SDL_RenderDrawLine(renderer, x + 3, y + 8, x + 6, y + 12);
        SDL_RenderDrawLine(renderer, x + 6, y + 12, x + 13, y + 4);
    }
    
    DrawText(fontSmall, text, x + boxSize + 5, y, textColor);
}

void DrawRadioButton(int x, int y, const char* text, bool selected, bool enabled) {
    int radius = 7;
    uint32_t textColor = enabled ? COLOR_BLACK : 0x808080FF;
    
    // Draw circle (approximate with rect for simplicity)
    DrawRect(x, y, radius * 2, radius * 2, COLOR_GRAY);
    DrawFilledRect(x + 1, y + 1, radius * 2 - 2, radius * 2 - 2, enabled ? COLOR_WHITE : 0xE0E0E0FF);
    
    if (selected) {
        DrawFilledRect(x + 4, y + 4, radius * 2 - 8, radius * 2 - 8, COLOR_BLACK);
    }
    
    DrawText(fontSmall, text, x + radius * 2 + 5, y, textColor);
}

void DrawComboBox(int x, int y, int w, const char* text, bool enabled) {
    uint32_t bgColor = enabled ? COLOR_WHITE : 0xE0E0E0FF;
    uint32_t textColor = enabled ? COLOR_BLACK : 0x808080FF;
    
    DrawFilledRect(x, y, w, 22, bgColor);
    DrawRect(x, y, w, 22, COLOR_GRAY);
    DrawText(fontSmall, text, x + 5, y + 3, textColor);
    
    // Arrow
    DrawFilledRect(x + w - 18, y + 1, 17, 20, 0xE0E0E0FF);
    DrawText(fontSmall, "v", x + w - 14, y + 3, textColor);
}

// ============================================================================
// DRAWING - CONSUMER READS FROM FRONT BUFFER
// ============================================================================

void DrawDistanceGraph(const DataBuffer* rb) {
    // Dynamic dimensions based on window size
    int leftPanelW = getLeftPanelWidth();
    int graphLeft = leftPanelW + 50;  // Extra padding between panel and graph
    int graphTop = 30;
    int graphWidth = windowWidth - graphLeft - 20;
    // Graph takes most of the height, stat boxes are fixed STAT_BOX_HEIGHT
    int graphHeight = windowHeight - graphTop - STAT_BOX_HEIGHT - 50;  // Leave space for stat boxes
    
    if (graphHeight < 150) graphHeight = 150;  // Minimum graph height
    if (graphWidth < 200) graphWidth = 200;    // Minimum graph width
    
    // Background
    DrawFilledRect(graphLeft, graphTop, graphWidth, graphHeight, COLOR_WHITE);
    DrawRect(graphLeft, graphTop, graphWidth, graphHeight, COLOR_DARK_GRAY);
    
    // Title
    DrawText(fontMedium, "Distance vs Time", graphLeft + 10, graphTop - 25, COLOR_BLACK);
    
    // Grid lines
    for (int i = 0; i <= 5; i++) {
        int y = graphTop + (graphHeight * i) / 5;
        SetColor(renderer, COLOR_LIGHT_GRAY);
        for (int x = graphLeft; x < graphLeft + graphWidth; x += 4) {
            SDL_RenderDrawPoint(renderer, x, y);
        }
    }
    
    // Y-axis labels
    char label[32];
    snprintf(label, sizeof(label), "%d cm", graphMaxRange);
    DrawText(fontSmall, label, graphLeft - 50, graphTop - 5, COLOR_DARK_GRAY);
    
    snprintf(label, sizeof(label), "%d cm", graphMaxRange / 2);
    DrawText(fontSmall, label, graphLeft - 50, graphTop + graphHeight / 2 - 5, COLOR_DARK_GRAY);
    
    DrawText(fontSmall, "0 cm", graphLeft - 40, graphTop + graphHeight - 10, COLOR_DARK_GRAY);
    
    // Draw data line
    if (rb->count > 1) {
        SetColor(renderer, COLOR_BLACK);
        
        int startIdx = (rb->head - rb->count + MAX_DATA_POINTS) % MAX_DATA_POINTS;
        int prevX = -1, prevY = -1;
        
        for (int i = 0; i < rb->count; i++) {
            int idx = (startIdx + i) % MAX_DATA_POINTS;
            int x = graphLeft + (graphWidth * i) / MAX_DATA_POINTS;
            
            int dist = rb->data[idx].distance;
            if (dist > graphMaxRange) dist = graphMaxRange;
            int y = graphTop + graphHeight - (graphHeight * dist) / graphMaxRange;
            
            if (prevX >= 0) {
                SDL_RenderDrawLine(renderer, prevX, prevY, x, y);
            }
            prevX = x;
            prevY = y;
        }
    }
}

void DrawLatencyBox(int x, int y, int w, int h, const char* title,
                    float avg, uint32_t count, uint32_t min, uint32_t max) {
    DrawFilledRect(x, y, w, h, COLOR_WHITE);
    DrawRect(x, y, w, h, COLOR_GRAY);
    
    // Title
    DrawText(fontMedium, title, x + 10, y + 8, COLOR_BLACK);
    
    // Value
    char val[32];
    if (count > 0) {
        snprintf(val, sizeof(val), "%.1f ms", avg);
    } else {
        snprintf(val, sizeof(val), "N/A");
    }
    DrawText(fontLarge, val, x + 10, y + 35, COLOR_BLACK);
    
    // Stats
    snprintf(val, sizeof(val), "Samples: %u", count);
    DrawText(fontSmall, val, x + 10, y + 75, COLOR_DARK_GRAY);
    
    if (count > 0) {
        snprintf(val, sizeof(val), "Min: %u ms", min);
        DrawText(fontSmall, val, x + 10, y + 95, COLOR_DARK_GRAY);
        
        snprintf(val, sizeof(val), "Max: %u ms", max);
        DrawText(fontSmall, val, x + 10, y + 115, COLOR_DARK_GRAY);
        
        snprintf(val, sizeof(val), "Avg: %.1f ms", avg);
        DrawText(fontSmall, val, x + 10, y + 135, COLOR_DARK_GRAY);
    } else {
        DrawText(fontSmall, "Min: --", x + 10, y + 95, COLOR_DARK_GRAY);
        DrawText(fontSmall, "Max: --", x + 10, y + 115, COLOR_DARK_GRAY);
        DrawText(fontSmall, "Avg: --", x + 10, y + 135, COLOR_DARK_GRAY);
    }
}

void DrawLatencyBoxes(const DataBuffer* rb) {
    // Dynamic dimensions based on window size
    int leftPanelW = getLeftPanelWidth();
    int graphLeft = leftPanelW + 50;  // Same padding as graph
    int graphWidth = windowWidth - graphLeft - 20;
    if (graphWidth < 200) graphWidth = 200;
    int graphHeight = windowHeight - 30 - STAT_BOX_HEIGHT - 50;
    if (graphHeight < 150) graphHeight = 150;
    
    int boxTop = 30 + graphHeight + 20;  // Below graph with margin
    int boxHeight = STAT_BOX_HEIGHT;     // Fixed height for stat boxes
    int boxWidth = graphWidth / 3;
    
    float freertosAvg = rb->freertosLatencyCount > 0 ? 
        (float)rb->freertosLatencySum / rb->freertosLatencyCount : 0;
    float baremetalAvg = rb->baremetalLatencyCount > 0 ? 
        (float)rb->baremetalLatencySum / rb->baremetalLatencyCount : 0;
    float ratio = (baremetalAvg > 0 && freertosAvg > 0) ? freertosAvg / baremetalAvg : 0;
    
    // FreeRTOS box
    DrawLatencyBox(graphLeft, boxTop, boxWidth, boxHeight, "FreeRTOS",
                   freertosAvg, rb->freertosLatencyCount, 
                   rb->freertosLatencyMin, rb->freertosLatencyMax);
    
    // BareMetal box
    DrawLatencyBox(graphLeft + boxWidth, boxTop, boxWidth, boxHeight, "BareMetal",
                   baremetalAvg, rb->baremetalLatencyCount,
                   rb->baremetalLatencyMin, rb->baremetalLatencyMax);
    
    // Ratio + Distance box
    int box3X = graphLeft + 2 * boxWidth;
    int halfHeight = boxHeight / 2;
    
    // Ratio
    DrawFilledRect(box3X, boxTop, boxWidth, halfHeight, COLOR_WHITE);
    DrawRect(box3X, boxTop, boxWidth, halfHeight, COLOR_GRAY);
    DrawText(fontMedium, "Ratio", box3X + 10, boxTop + 8, COLOR_BLACK);
    
    char val[32];
    if (ratio > 0) {
        snprintf(val, sizeof(val), "%.2fx", ratio);
    } else {
        snprintf(val, sizeof(val), "N/A");
    }
    DrawText(fontLarge, val, box3X + 10, boxTop + 35, COLOR_BLACK);
    
    // Distance
    DrawFilledRect(box3X, boxTop + halfHeight, boxWidth, halfHeight, COLOR_WHITE);
    DrawRect(box3X, boxTop + halfHeight, boxWidth, halfHeight, COLOR_GRAY);
    DrawText(fontMedium, "Distance", box3X + 10, boxTop + halfHeight + 8, COLOR_BLACK);
    
    snprintf(val, sizeof(val), "%u cm", rb->currentDistance);
    DrawText(fontLarge, val, box3X + 10, boxTop + halfHeight + 35, COLOR_BLACK);
}

void DrawLeftPanel(const DataBuffer* rb) {
    int panelW = getLeftPanelWidth();
    int contentW = panelW - 20;  // 10px margin each side
    int y = 10;
    
    // Port Setup Frame
    DrawRect(10, y, contentW, 110, COLOR_GRAY);
    DrawText(fontSmall, " Port Setup ", 15, y - 8, COLOR_BLACK);
    
    DrawText(fontSmall, "Port:", 20, y + 20, COLOR_BLACK);
    const char* portText = portCount > 0 ? availablePorts[selectedPort] : "No ports";
    DrawComboBox(70, y + 17, contentW - 80, portText, !isConnected);
    
    DrawText(fontSmall, "Baud:", 20, y + 50, COLOR_BLACK);
    const char* baudText = selectedBaud == 0 ? "9600" : "115200";
    DrawComboBox(70, y + 47, contentW - 80, baudText, !isConnected);
    
    int btnW = (contentW - 30) / 2;
    DrawButton(20, y + 78, btnW, 25, "Connect", !isConnected && selectedMode >= 0, false);
    DrawButton(20 + btnW + 5, y + 78, btnW, 25, "Disconnect", isConnected, false);
    
    y += 120;
    
    // Run Mode Frame
    DrawRect(10, y, contentW, 50, COLOR_GRAY);
    DrawText(fontSmall, " Run Mode ", 15, y - 8, COLOR_BLACK);
    
    DrawRadioButton(20, y + 20, "BareMetal", selectedMode == 0, !isConnected);
    DrawRadioButton(20 + contentW/2, y + 20, "FreeRTOS", selectedMode == 1, !isConnected);
    
    y += 60;
    
    // Graph Range Frame
    DrawRect(10, y, contentW, 50, COLOR_GRAY);
    DrawText(fontSmall, " Graph Range ", 15, y - 8, COLOR_BLACK);
    
    DrawText(fontSmall, "0 -", 20, y + 22, COLOR_BLACK);
    // Input field with active highlight
    uint32_t rangeBoxColor = (activeInputField == 1) ? 0xFFFFCCFF : COLOR_WHITE;
    uint32_t rangeBorderColor = (activeInputField == 1) ? COLOR_BLUE : COLOR_GRAY;
    DrawFilledRect(50, y + 18, 50, 22, rangeBoxColor);
    DrawRect(50, y + 18, 50, 22, rangeBorderColor);
    if (activeInputField == 1) {
        DrawRect(49, y + 17, 52, 24, rangeBorderColor);
    }
    DrawText(fontSmall, rangeInputText, 55, y + 21, COLOR_BLACK);
    DrawText(fontSmall, "cm", 105, y + 22, COLOR_BLACK);
    DrawButton(contentW - 50, y + 18, 55, 24, "Apply", true, false);
    
    y += 60;
    
    // Scheduling Overhead Frame
    DrawRect(10, y, contentW, 45, COLOR_GRAY);
    DrawText(fontSmall, " Scheduling Overhead ", 15, y - 8, COLOR_BLACK);
    
    DrawText(fontSmall, "Status:", 20, y + 18, COLOR_BLACK);
    const char* overheadText = schedulingOverheadEnabled ? "Active" : "Inactive";
    DrawComboBox(80, y + 14, contentW - 90, overheadText, selectedMode == 1);
    
    y += 55;
    
    // Critical Alert Frame
    DrawRect(10, y, contentW, 90, COLOR_GRAY);
    DrawText(fontSmall, " Critical Alert ", 15, y - 8, COLOR_BLACK);
    
    DrawText(fontSmall, "Threshold:", 20, y + 18, COLOR_BLACK);
    // Input field with active highlight
    uint32_t threshBoxColor = (activeInputField == 2) ? 0xFFFFCCFF : (selectedMode == 1 ? COLOR_WHITE : 0xE0E0E0FF);
    uint32_t threshBorderColor = (activeInputField == 2) ? COLOR_BLUE : COLOR_GRAY;
    DrawFilledRect(95, y + 15, 40, 22, threshBoxColor);
    DrawRect(95, y + 15, 40, 22, threshBorderColor);
    if (activeInputField == 2) {
        DrawRect(94, y + 14, 42, 24, threshBorderColor);
    }
    DrawText(fontSmall, thresholdInputText, 100, y + 18, selectedMode == 1 ? COLOR_BLACK : 0x808080FF);
    DrawText(fontSmall, "cm", 140, y + 18, COLOR_BLACK);
    
    // Set button with press feedback
    bool setPressed = setButtonPressed && (SDL_GetTicks() - setButtonPressTime < 200);
    DrawButton(contentW - 35, y + 15, 40, 24, "Set", selectedMode == 1, setPressed);
    
    // Alert status
    if (rb->alertActive) {
        char alertText[64];
        snprintf(alertText, sizeof(alertText), "ALERT! Samples: %u", rb->alertSamples);
        DrawText(fontSmall, alertText, 20, y + 42, COLOR_RED);
        snprintf(alertText, sizeof(alertText), "Duration: %u ms", rb->alertDuration);
        DrawText(fontSmall, alertText, 20, y + 58, COLOR_RED);
    } else {
        DrawText(fontSmall, "Status: OK", 20, y + 45, COLOR_GREEN);
    }
    
    DrawButton(20, y + 65, contentW - 20, 20, "Acknowledge", rb->alertActive, false);
    
    y += 100;
    
    // Options Frame
    DrawRect(10, y, contentW, 65, COLOR_GRAY);
    DrawText(fontSmall, " Options ", 15, y - 8, COLOR_BLACK);
    
    DrawCheckbox(20, y + 18, "Enable Logging", logEnabled, true);
    DrawButton(20, y + 40, contentW - 20, 22, "Reset Stats", true, false);
    
    y += 75;
    
    // Status Bar
    DrawFilledRect(10, y, contentW, 22, 0xE0E0E0FF);
    DrawRect(10, y, contentW, 22, COLOR_GRAY);
    
    const char* statusText = "Select Run Mode";
    if (isConnected) {
        statusText = "Connected";
    } else if (selectedMode == 0) {
        statusText = "BareMetal Selected";
    } else if (selectedMode == 1) {
        statusText = "FreeRTOS Selected";
    }
    DrawText(fontSmall, statusText, 15, y + 3, COLOR_BLACK);
}

// ============================================================================
// EVENT HANDLING
// ============================================================================

bool PointInRect(int px, int py, int x, int y, int w, int h) {
    return px >= x && px < x + w && py >= y && py < y + h;
}

void HandleMouseClick(int mx, int my) {
    int panelW = getLeftPanelWidth();
    int contentW = panelW - 20;
    int btnW = (contentW - 30) / 2;
    int y = 10;
    
    // Connect button
    if (PointInRect(mx, my, 20, y + 78, btnW, 25)) {
        if (!isConnected && selectedMode >= 0 && portCount > 0) {
            int baud = selectedBaud == 0 ? 9600 : 115200;
            if (OpenSerialPort(availablePorts[selectedPort], baud)) {
                isConnected = true;
                bRunning = true;
                connectedMode = selectedMode == 0 ? 'B' : 'F';
                
                // Reset stats
                DataBuffer* wb = DoubleBuffer_GetWriteBuffer(&doubleBuffer);
                if (connectedMode == 'F') {
                    wb->freertosLatencySum = 0;
                    wb->freertosLatencyCount = 0;
                    wb->freertosLatencyMin = 0xFFFFFFFF;
                    wb->freertosLatencyMax = 0;
                } else {
                    wb->baremetalLatencySum = 0;
                    wb->baremetalLatencyCount = 0;
                    wb->baremetalLatencyMin = 0xFFFFFFFF;
                    wb->baremetalLatencyMax = 0;
                }
                wb->count = 0;
                wb->head = 0;
                DoubleBuffer_Swap(&doubleBuffer);
                
                pthread_create(&producerThread, NULL, ProducerThread, NULL);
                
                usleep(100000);
                char cmd = connectedMode;
                SendSerialData(&cmd, 1);
            }
        }
    }
    
    // Disconnect button (dynamic position)
    if (PointInRect(mx, my, 20 + btnW + 5, y + 78, btnW, 25)) {
        if (isConnected) {
            bRunning = false;
            pthread_join(producerThread, NULL);
            CloseSerialPort();
            isConnected = false;
            connectedMode = '?';
        }
    }
    
    // Port combo (cycle through)
    if (PointInRect(mx, my, 70, y + 17, contentW - 80, 22) && !isConnected) {
        selectedPort = (selectedPort + 1) % portCount;
    }
    
    // Baud combo
    if (PointInRect(mx, my, 70, y + 47, contentW - 80, 22) && !isConnected) {
        selectedBaud = 1 - selectedBaud;
    }
    
    y += 120;
    
    // BareMetal radio
    if (PointInRect(mx, my, 20, y + 20, contentW/2 - 10, 20) && !isConnected) {
        selectedMode = 0;
    }
    
    // FreeRTOS radio (dynamic position)
    if (PointInRect(mx, my, 20 + contentW/2, y + 20, contentW/2 - 10, 20) && !isConnected) {
        selectedMode = 1;
    }
    
    y += 60;
    
    // Range input field click - activate text input
    if (PointInRect(mx, my, 50, y + 18, 50, 22)) {
        activeInputField = 1;
        SDL_StartTextInput();
    }
    // Apply Range button (dynamic position)
    else if (PointInRect(mx, my, contentW - 50, y + 18, 55, 24)) {
        int val = atoi(rangeInputText);
        if (val < 5) val = 5;
        if (val > 1000) val = 1000;
        graphMaxRange = val;
        snprintf(rangeInputText, sizeof(rangeInputText), "%d", val);
        activeInputField = 0;
        SDL_StopTextInput();
    }
    // Click outside input fields - deactivate
    else if (activeInputField == 1) {
        activeInputField = 0;
        SDL_StopTextInput();
    }
    
    y += 60;
    
    // Scheduling Overhead toggle
    if (PointInRect(mx, my, 80, y + 14, contentW - 90, 22) && selectedMode == 1) {
        schedulingOverheadEnabled = !schedulingOverheadEnabled;
        if (schedulingOverheadEnabled) {
            overheadBufferHead = 0;
            overheadBufferCount = 0;
        }
    }
    
    y += 55;
    
    // Threshold input field click - activate text input
    if (PointInRect(mx, my, 95, y + 15, 40, 22) && selectedMode == 1) {
        activeInputField = 2;
        SDL_StartTextInput();
    }
    // Set Threshold button (dynamic position)
    else if (PointInRect(mx, my, contentW - 35, y + 15, 40, 24) && selectedMode == 1) {
        // Parse threshold and send command - only if text is not empty
        if (strlen(thresholdInputText) > 0) {
            int val = atoi(thresholdInputText);
            if (val < 5) val = 5;
            if (val > 30) val = 30;
            currentThreshold = val;
            thresholdEnabled = true;  // Enable threshold after user sets it
            snprintf(thresholdInputText, sizeof(thresholdInputText), "%d", val);
            
            // Visual feedback
            setButtonPressed = true;
            setButtonPressTime = SDL_GetTicks();
            
            // Send command to STM32 if connected
            if (serialFd >= 0) {
                char cmd;
                if (val <= 5) { cmd = 'T'; val = 5; }
                else if (val <= 10) { cmd = 'U'; val = 10; }
                else if (val <= 15) { cmd = 'V'; val = 15; }
                else if (val <= 20) { cmd = 'W'; val = 20; }
                else if (val <= 25) { cmd = 'X'; val = 25; }
                else { cmd = 'Y'; val = 30; }
                
                ssize_t ret = write(serialFd, &cmd, 1);
                (void)ret;  // Suppress warning
            }
        }
        
        activeInputField = 0;
        SDL_StopTextInput();
    }
    // Click outside threshold input - deactivate
    else if (activeInputField == 2) {
        activeInputField = 0;
        SDL_StopTextInput();
    }
    
    // Acknowledge button (dynamic width)
    if (PointInRect(mx, my, 20, y + 65, contentW - 20, 20)) {
        DataBuffer* rb = DoubleBuffer_GetReadBuffer(&doubleBuffer);
        if (rb->alertActive && serialFd >= 0) {
            char cmd = 'A';
            ssize_t ret = write(serialFd, &cmd, 1);
            (void)ret;
        }
    }
    
    y += 100;
    
    // Log checkbox
    if (PointInRect(mx, my, 20, y + 18, 150, 20)) {
        logEnabled = !logEnabled;
    }
    
    // Reset Stats button
    if (PointInRect(mx, my, 20, y + 40, 180, 22)) {
        DataBuffer* wb = DoubleBuffer_GetWriteBuffer(&doubleBuffer);
        wb->freertosLatencySum = 0;
        wb->freertosLatencyCount = 0;
        wb->freertosLatencyMin = 0xFFFFFFFF;
        wb->freertosLatencyMax = 0;
        wb->baremetalLatencySum = 0;
        wb->baremetalLatencyCount = 0;
        wb->baremetalLatencyMin = 0xFFFFFFFF;
        wb->baremetalLatencyMax = 0;
        wb->count = 0;
        wb->head = 0;
        DoubleBuffer_Swap(&doubleBuffer);
        WriteLog("INFO", "Statistics reset by user");
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;
    
    // Initialize
    DoubleBuffer_Init(&doubleBuffer);
    InitLogFile();
    WriteLog("INFO", "Application started");
    EnumerateSerialPorts();
    
    // SDL init
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    
    if (TTF_Init() < 0) {
        fprintf(stderr, "TTF_Init failed: %s\n", TTF_GetError());
        SDL_Quit();
        return 1;
    }
    
    window = SDL_CreateWindow(
        "STM32 Real-Time Sensor Monitor (Linux SDL2)",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );
    
    // Set minimum window size
    SDL_SetWindowMinimumSize(window, WINDOW_WIDTH, WINDOW_HEIGHT);
    
    if (!window) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        TTF_Quit();
        SDL_Quit();
        return 1;
    }
    
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        TTF_Quit();
        SDL_Quit();
        return 1;
    }
    
    // Load fonts
    const char* fontPaths[] = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        NULL
    };
    
    for (int i = 0; fontPaths[i] != NULL; i++) {
        fontSmall = TTF_OpenFont(fontPaths[i], 12);
        if (fontSmall) {
            fontMedium = TTF_OpenFont(fontPaths[i], 14);
            fontLarge = TTF_OpenFont(fontPaths[i], 20);
            break;
        }
    }
    
    if (!fontSmall) {
        fprintf(stderr, "Failed to load any font. Install fonts with:\n");
        fprintf(stderr, "  sudo apt install fonts-dejavu-core\n");
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        TTF_Quit();
        SDL_Quit();
        return 1;
    }
    
    // Main loop
    bool running = true;
    SDL_Event event;
    Uint32 lastUpdate = SDL_GetTicks();
    
    while (running) {
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    running = false;
                    break;
                
                case SDL_WINDOWEVENT:
                    if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                        windowWidth = event.window.data1;
                        windowHeight = event.window.data2;
                    }
                    break;
                    
                case SDL_MOUSEBUTTONDOWN:
                    if (event.button.button == SDL_BUTTON_LEFT) {
                        HandleMouseClick(event.button.x, event.button.y);
                    }
                    break;
                
                case SDL_KEYDOWN:
                    if (activeInputField > 0) {
                        char* inputText = (activeInputField == 1) ? rangeInputText : thresholdInputText;
                        int len = strlen(inputText);
                        
                        if (event.key.keysym.sym == SDLK_BACKSPACE && len > 0) {
                            inputText[len - 1] = '\0';
                        } else if (event.key.keysym.sym == SDLK_RETURN || event.key.keysym.sym == SDLK_KP_ENTER) {
                            // Apply value on Enter
                            if (activeInputField == 1) {
                                int val = atoi(rangeInputText);
                                if (val < 5) val = 5;
                                if (val > 1000) val = 1000;
                                graphMaxRange = val;
                                snprintf(rangeInputText, sizeof(rangeInputText), "%d", val);
                            } else if (activeInputField == 2) {
                                int val = atoi(thresholdInputText);
                                if (val < 5) val = 5;
                                if (val > 30) val = 30;
                                currentThreshold = val;
                                snprintf(thresholdInputText, sizeof(thresholdInputText), "%d", val);
                            }
                            activeInputField = 0;
                        } else if (event.key.keysym.sym == SDLK_ESCAPE) {
                            activeInputField = 0;
                        }
                    }
                    break;
                
                case SDL_TEXTINPUT:
                    if (activeInputField > 0) {
                        char* inputText = (activeInputField == 1) ? rangeInputText : thresholdInputText;
                        int len = strlen(inputText);
                        // Only allow digits, max 4 chars
                        if (len < 4 && event.text.text[0] >= '0' && event.text.text[0] <= '9') {
                            inputText[len] = event.text.text[0];
                            inputText[len + 1] = '\0';
                        }
                    }
                    break;
            }
        }
        
        Uint32 now = SDL_GetTicks();
        if (now - lastUpdate >= GRAPH_UPDATE_MS) {
            lastUpdate = now;
            
            // Clear
            SetColor(renderer, COLOR_BG);
            SDL_RenderClear(renderer);
            
            // Get read buffer (consumer reads from front buffer)
            const DataBuffer* rb = DoubleBuffer_GetReadBuffer(&doubleBuffer);
            
            // Draw UI
            DrawLeftPanel(rb);
            DrawDistanceGraph(rb);
            DrawLatencyBoxes(rb);
            
            // Present
            SDL_RenderPresent(renderer);
        }
        
        SDL_Delay(10);  // ~100 FPS cap
    }
    
    // Cleanup
    
    if (isConnected) {
        bRunning = false;
        pthread_join(producerThread, NULL);
        CloseSerialPort();
    }
    
    CloseLogFile();
    
    if (fontLarge) TTF_CloseFont(fontLarge);
    if (fontMedium) TTF_CloseFont(fontMedium);
    if (fontSmall) TTF_CloseFont(fontSmall);
    
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
    
    return 0;
}
