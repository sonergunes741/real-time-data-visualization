/*
 * STM32 Real-Time Sensor Monitor
 * Linux PREEMPT_RT Version - GTK+ 3 UI
 * 
 * Compile: gcc -Wall -O2 -o sensor_monitor sensor_monitor_linux.c `pkg-config --cflags --libs gtk+-3.0` -lpthread
 */

#define _GNU_SOURCE
#include <gtk/gtk.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sched.h>
#include <dirent.h>

// ============================================================================
// DEFINITIONS
// ============================================================================

#define MAX_DATA_POINTS 300
#define GRAPH_UPDATE_MS 100
#define MAX_LOG_LINES 100

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// GTK Widgets
GtkWidget *mainWindow;
GtkWidget *comboPort, *comboBaud;
GtkWidget *btnConnect, *btnDisconnect;
GtkWidget *radioFreeRTOS, *radioBareMetal;
GtkWidget *editRange;
GtkWidget *btnApplyRange;
GtkWidget *comboBusyWait;
GtkWidget *btnResetStats;
GtkWidget *checkLog;
GtkWidget *statusBar;
GtkWidget *drawingArea;
// Alert section
GtkWidget *alertStatus;
GtkWidget *btnAckAlert;
GtkWidget *editThreshold;
GtkWidget *btnSetThresh;

int serialFd = -1;
pthread_t readThread;
volatile bool bRunning = false;

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
pthread_mutex_t dataMutex = PTHREAD_MUTEX_INITIALIZER;

// Current values
char currentMode = '?';
uint32_t currentDistance = 0;

// Distance averaging
uint32_t distanceSum = 0;
uint32_t distanceCount_val = 0;
uint32_t lastValidDistance = 0;

// Busy-Wait simulation (0.5s delay buffer)
bool busyWaitEnabled = false;
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
bool logEnabled = false;
FILE* logFile = NULL;
int logLineCount = 0;

// Alert state (from STM32)
bool alertActive = false;
uint32_t alertSamples = 0;
uint32_t alertDuration = 0;
int alertThreshold = 0;

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

void InitLogFile() {
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
    if (!logFile || logLineCount >= MAX_LOG_LINES) return;
    
    char timestamp[32];
    get_timestamp_str(timestamp, sizeof(timestamp));
    fprintf(logFile, "[%s] %s: %s\n", timestamp, prefix, msg);
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

int get_baud_constant(int baudRate) {
    switch (baudRate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
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
    
    // Configure serial port
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serialFd, &tty) != 0) {
        close(serialFd);
        serialFd = -1;
        return false;
    }
    
    // Set baud rate
    int baudConstant = get_baud_constant(baudRate);
    cfsetospeed(&tty, baudConstant);
    cfsetispeed(&tty, baudConstant);
    
    // 8N1 mode
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control
    
    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    
    // Read timeout settings
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // 100ms timeout
    
    if (tcsetattr(serialFd, TCSANOW, &tty) != 0) {
        close(serialFd);
        serialFd = -1;
        return false;
    }
    
    // Flush buffers
    tcflush(serialFd, TCIOFLUSH);
    
    // Set DTR and RTS
    int status;
    ioctl(serialFd, TIOCMGET, &status);
    status |= TIOCM_DTR | TIOCM_RTS;
    ioctl(serialFd, TIOCMSET, &status);
    
    char msg[64];
    snprintf(msg, sizeof(msg), "Connected to %s @ %d", portName, baudRate);
    WriteLog("INFO", msg);
    
    return true;
}

void CloseSerialPort() {
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
        alertActive = true;
        char* samplesPtr = strstr(line, "SAMPLES:");
        char* durationPtr = strstr(line, "DURATION:");
        char* threshPtr = strstr(line, "THRESHOLD:");
        
        if (samplesPtr) alertSamples = atol(samplesPtr + 8);
        if (durationPtr) alertDuration = atol(durationPtr + 9);
        if (threshPtr) alertThreshold = atoi(threshPtr + 10);
        return;
    }
    
    if (strstr(line, "ALERT:CLEARED") != NULL) {
        alertActive = false;
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
        distanceCount_val++;
        lastValidDistance = data.distance;
        
        pthread_mutex_lock(&dataMutex);
        
        if (busyWaitEnabled) {
            // Buffer data with delay
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
        
        pthread_mutex_unlock(&dataMutex);
    }
}

// ============================================================================
// SERIAL READ THREAD (PREEMPT_RT Optimized)
// ============================================================================

void* SerialReadThread(void* arg) {
    (void)arg;
    
    // Set real-time scheduling for PREEMPT_RT kernel
    struct sched_param param;
    param.sched_priority = 80;  // High priority for sensor reading
    
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        // Not fatal - will work without RT scheduling
        WriteLog("WARN", "Could not set RT scheduling (run as root for RT priority)");
    }
    
    char buffer[256];
    char lineBuffer[256];
    int linePos = 0;
    
    WriteLog("INFO", "Read thread started");
    
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
        usleep(10000);  // 10ms sleep
    }
    
    return NULL;
}

// ============================================================================
// DRAWING FUNCTIONS
// ============================================================================

gboolean on_draw(GtkWidget *widget, cairo_t *cr, gpointer data) {
    (void)data;
    
    GtkAllocation alloc;
    gtk_widget_get_allocation(widget, &alloc);
    
    int totalWidth = alloc.width;
    int totalHeight = alloc.height;
    
    // Background
    cairo_set_source_rgb(cr, 0.95, 0.95, 0.95);
    cairo_paint(cr);
    
    // ==================== DISTANCE GRAPH ====================
    int graphLeft = 60;
    int graphTop = 30;
    int graphRight = totalWidth - 20;
    int graphBottom = totalHeight / 2 - 20;
    int graphWidth = graphRight - graphLeft;
    int graphHeight = graphBottom - graphTop;
    
    // Graph background
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_rectangle(cr, graphLeft, graphTop, graphWidth, graphHeight);
    cairo_fill(cr);
    
    // Graph border
    cairo_set_source_rgb(cr, 0.4, 0.4, 0.4);
    cairo_set_line_width(cr, 2);
    cairo_rectangle(cr, graphLeft, graphTop, graphWidth, graphHeight);
    cairo_stroke(cr);
    
    // Title
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 16);
    cairo_move_to(cr, graphLeft, graphTop - 10);
    cairo_show_text(cr, "Distance vs Time");
    
    // Grid lines
    cairo_set_source_rgb(cr, 0.85, 0.85, 0.85);
    cairo_set_line_width(cr, 1);
    double dashes[] = {4.0, 4.0};
    cairo_set_dash(cr, dashes, 2, 0);
    for (int i = 0; i <= 5; i++) {
        int y = graphTop + (graphHeight * i) / 5;
        cairo_move_to(cr, graphLeft, y);
        cairo_line_to(cr, graphRight, y);
        cairo_stroke(cr);
    }
    cairo_set_dash(cr, NULL, 0, 0);
    
    // Y-axis labels
    cairo_set_source_rgb(cr, 0.3, 0.3, 0.3);
    cairo_set_font_size(cr, 12);
    char label[32];
    
    snprintf(label, sizeof(label), "%d cm", graphMaxRange);
    cairo_move_to(cr, 5, graphTop + 5);
    cairo_show_text(cr, label);
    
    snprintf(label, sizeof(label), "%d cm", graphMaxRange / 2);
    cairo_move_to(cr, 5, graphTop + graphHeight / 2 + 5);
    cairo_show_text(cr, label);
    
    cairo_move_to(cr, 5, graphBottom);
    cairo_show_text(cr, "0 cm");
    
    // Draw data line
    pthread_mutex_lock(&dataMutex);
    
    if (dataCount > 1) {
        cairo_set_source_rgb(cr, 0, 0, 0);
        cairo_set_line_width(cr, 2);
        
        int startIdx = (dataHead - dataCount + MAX_DATA_POINTS) % MAX_DATA_POINTS;
        bool firstPoint = true;
        
        for (int i = 0; i < dataCount; i++) {
            int idx = (startIdx + i) % MAX_DATA_POINTS;
            int x = graphLeft + (graphWidth * i) / MAX_DATA_POINTS;
            
            int dist = dataBuffer[idx].distance;
            if (dist > graphMaxRange) dist = graphMaxRange;
            int y = graphBottom - (graphHeight * dist) / graphMaxRange;
            
            if (firstPoint) {
                cairo_move_to(cr, x, y);
                firstPoint = false;
            } else {
                cairo_line_to(cr, x, y);
            }
        }
        cairo_stroke(cr);
    }
    
    pthread_mutex_unlock(&dataMutex);
    
    // ==================== LATENCY BOXES ====================
    pthread_mutex_lock(&dataMutex);
    
    float freertosAvg = freertosLatencyCount > 0 ? 
        (float)freertosLatencySum / freertosLatencyCount : 0;
    float baremetalAvg = baremetalLatencyCount > 0 ? 
        (float)baremetalLatencySum / baremetalLatencyCount : 0;
    float ratio = (baremetalAvg > 0 && freertosAvg > 0) ? freertosAvg / baremetalAvg : 0;
    
    uint32_t fCount = freertosLatencyCount;
    uint32_t fMin = freertosLatencyMin;
    uint32_t fMax = freertosLatencyMax;
    uint32_t bCount = baremetalLatencyCount;
    uint32_t bMin = baremetalLatencyMin;
    uint32_t bMax = baremetalLatencyMax;
    uint32_t curDist = currentDistance;
    
    pthread_mutex_unlock(&dataMutex);
    
    int boxTop = totalHeight / 2;
    int boxBottom = totalHeight - 10;
    int boxHeight = boxBottom - boxTop;
    int boxWidth = (totalWidth - 40) / 3;
    int halfBoxHeight = boxHeight / 2;
    
    // Box 1: FreeRTOS
    int box1Left = 20;
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_rectangle(cr, box1Left, boxTop, boxWidth, boxHeight);
    cairo_fill(cr);
    cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
    cairo_set_line_width(cr, 1);
    cairo_rectangle(cr, box1Left, boxTop, boxWidth, boxHeight);
    cairo_stroke(cr);
    
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 14);
    cairo_move_to(cr, box1Left + 10, boxTop + 20);
    cairo_show_text(cr, "FreeRTOS");
    
    cairo_set_font_size(cr, 24);
    if (fCount > 0) {
        snprintf(label, sizeof(label), "%.1f ms", freertosAvg);
    } else {
        snprintf(label, sizeof(label), "N/A");
    }
    cairo_move_to(cr, box1Left + 10, boxTop + 55);
    cairo_show_text(cr, label);
    
    cairo_set_source_rgb(cr, 0.3, 0.3, 0.3);
    cairo_set_font_size(cr, 12);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    
    snprintf(label, sizeof(label), "Samples: %u", fCount);
    cairo_move_to(cr, box1Left + 10, boxTop + 80);
    cairo_show_text(cr, label);
    
    if (fCount > 0) {
        snprintf(label, sizeof(label), "Min: %u ms", fMin);
        cairo_move_to(cr, box1Left + 10, boxTop + 100);
        cairo_show_text(cr, label);
        
        snprintf(label, sizeof(label), "Max: %u ms", fMax);
        cairo_move_to(cr, box1Left + 10, boxTop + 120);
        cairo_show_text(cr, label);
        
        snprintf(label, sizeof(label), "Avg: %.1f ms", freertosAvg);
        cairo_move_to(cr, box1Left + 10, boxTop + 140);
        cairo_show_text(cr, label);
    } else {
        cairo_move_to(cr, box1Left + 10, boxTop + 100);
        cairo_show_text(cr, "Min: --");
        cairo_move_to(cr, box1Left + 10, boxTop + 120);
        cairo_show_text(cr, "Max: --");
        cairo_move_to(cr, box1Left + 10, boxTop + 140);
        cairo_show_text(cr, "Avg: --");
    }
    
    // Box 2: BareMetal
    int box2Left = box1Left + boxWidth;
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_rectangle(cr, box2Left, boxTop, boxWidth, boxHeight);
    cairo_fill(cr);
    cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
    cairo_rectangle(cr, box2Left, boxTop, boxWidth, boxHeight);
    cairo_stroke(cr);
    
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 14);
    cairo_move_to(cr, box2Left + 10, boxTop + 20);
    cairo_show_text(cr, "BareMetal");
    
    cairo_set_font_size(cr, 24);
    if (bCount > 0) {
        snprintf(label, sizeof(label), "%.1f ms", baremetalAvg);
    } else {
        snprintf(label, sizeof(label), "N/A");
    }
    cairo_move_to(cr, box2Left + 10, boxTop + 55);
    cairo_show_text(cr, label);
    
    cairo_set_source_rgb(cr, 0.3, 0.3, 0.3);
    cairo_set_font_size(cr, 12);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    
    snprintf(label, sizeof(label), "Samples: %u", bCount);
    cairo_move_to(cr, box2Left + 10, boxTop + 80);
    cairo_show_text(cr, label);
    
    if (bCount > 0) {
        snprintf(label, sizeof(label), "Min: %u ms", bMin);
        cairo_move_to(cr, box2Left + 10, boxTop + 100);
        cairo_show_text(cr, label);
        
        snprintf(label, sizeof(label), "Max: %u ms", bMax);
        cairo_move_to(cr, box2Left + 10, boxTop + 120);
        cairo_show_text(cr, label);
        
        snprintf(label, sizeof(label), "Avg: %.1f ms", baremetalAvg);
        cairo_move_to(cr, box2Left + 10, boxTop + 140);
        cairo_show_text(cr, label);
    } else {
        cairo_move_to(cr, box2Left + 10, boxTop + 100);
        cairo_show_text(cr, "Min: --");
        cairo_move_to(cr, box2Left + 10, boxTop + 120);
        cairo_show_text(cr, "Max: --");
        cairo_move_to(cr, box2Left + 10, boxTop + 140);
        cairo_show_text(cr, "Avg: --");
    }
    
    // Box 3 Top: Ratio
    int box3Left = box2Left + boxWidth;
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_rectangle(cr, box3Left, boxTop, boxWidth, halfBoxHeight);
    cairo_fill(cr);
    cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
    cairo_rectangle(cr, box3Left, boxTop, boxWidth, halfBoxHeight);
    cairo_stroke(cr);
    
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 14);
    cairo_move_to(cr, box3Left + 10, boxTop + 20);
    cairo_show_text(cr, "Ratio");
    
    cairo_set_font_size(cr, 24);
    if (ratio > 0) {
        snprintf(label, sizeof(label), "%.2fx", ratio);
    } else {
        snprintf(label, sizeof(label), "N/A");
    }
    cairo_move_to(cr, box3Left + 10, boxTop + 55);
    cairo_show_text(cr, label);
    
    // Box 3 Bottom: Distance
    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_rectangle(cr, box3Left, boxTop + halfBoxHeight, boxWidth, halfBoxHeight);
    cairo_fill(cr);
    cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
    cairo_rectangle(cr, box3Left, boxTop + halfBoxHeight, boxWidth, halfBoxHeight);
    cairo_stroke(cr);
    
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 14);
    cairo_move_to(cr, box3Left + 10, boxTop + halfBoxHeight + 20);
    cairo_show_text(cr, "Distance");
    
    cairo_set_font_size(cr, 24);
    snprintf(label, sizeof(label), "%u cm", curDist);
    cairo_move_to(cr, box3Left + 10, boxTop + halfBoxHeight + 55);
    cairo_show_text(cr, label);
    
    return FALSE;
}

// ============================================================================
// GTK CALLBACKS
// ============================================================================

void update_status(const char* text) {
    gtk_label_set_text(GTK_LABEL(statusBar), text);
}

gboolean update_ui(gpointer data) {
    (void)data;
    
    // Update alert display
    if (alertActive) {
        char alertText[128];
        snprintf(alertText, sizeof(alertText), "ALERT! Samples: %u\nDuration: %u ms", 
                alertSamples, alertDuration);
        gtk_label_set_text(GTK_LABEL(alertStatus), alertText);
        gtk_widget_set_sensitive(btnAckAlert, TRUE);
    } else {
        gtk_label_set_text(GTK_LABEL(alertStatus), "Status: OK");
        gtk_widget_set_sensitive(btnAckAlert, FALSE);
    }
    
    // Redraw graph
    gtk_widget_queue_draw(drawingArea);
    
    return TRUE;  // Continue timer
}

void on_connect_clicked(GtkWidget *widget, gpointer data) {
    (void)widget;
    (void)data;
    
    // Mode selection check
    gboolean freertosSelected = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radioFreeRTOS));
    gboolean baremetalSelected = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radioBareMetal));
    
    if (!freertosSelected && !baremetalSelected) {
        update_status("Select Run Mode first!");
        return;
    }
    
    const char* port = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(comboPort));
    const char* baudStr = gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(comboBaud));
    int baud = atoi(baudStr);
    
    if (OpenSerialPort(port, baud)) {
        bRunning = true;
        pthread_create(&readThread, NULL, SerialReadThread, NULL);
        
        gtk_widget_set_sensitive(btnConnect, FALSE);
        gtk_widget_set_sensitive(btnDisconnect, TRUE);
        gtk_widget_set_sensitive(radioFreeRTOS, FALSE);
        gtk_widget_set_sensitive(radioBareMetal, FALSE);
        gtk_widget_set_sensitive(checkLog, FALSE);
        update_status("Connected");
        
        usleep(100000);  // 100ms delay
        
        // Set connected mode
        connectedMode = freertosSelected ? 'F' : 'B';
        
        // Reset stats for connected mode
        pthread_mutex_lock(&dataMutex);
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
        pthread_mutex_unlock(&dataMutex);
        
        // Enable FreeRTOS-only controls
        gboolean isFreeRTOS = (connectedMode == 'F');
        gtk_widget_set_sensitive(comboBusyWait, isFreeRTOS);
        gtk_widget_set_sensitive(editThreshold, isFreeRTOS);
        gtk_widget_set_sensitive(btnSetThresh, isFreeRTOS);
        
        // Send mode command
        char cmd = connectedMode;
        SendSerialData(&cmd, 1);
    } else {
        update_status("Connection Failed");
    }
}

void on_disconnect_clicked(GtkWidget *widget, gpointer data) {
    (void)widget;
    (void)data;
    
    bRunning = false;
    pthread_join(readThread, NULL);
    CloseSerialPort();
    
    connectedMode = '?';
    
    gtk_widget_set_sensitive(btnConnect, TRUE);
    gtk_widget_set_sensitive(btnDisconnect, FALSE);
    gtk_widget_set_sensitive(radioFreeRTOS, TRUE);
    gtk_widget_set_sensitive(radioBareMetal, TRUE);
    gtk_widget_set_sensitive(checkLog, TRUE);
    gtk_widget_set_sensitive(comboBusyWait, TRUE);
    gtk_widget_set_sensitive(editThreshold, TRUE);
    gtk_widget_set_sensitive(btnSetThresh, TRUE);
    update_status("Disconnected");
}

void on_radio_freertos_toggled(GtkWidget *widget, gpointer data) {
    (void)data;
    
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
        gtk_widget_set_sensitive(btnConnect, TRUE);
        gtk_widget_set_sensitive(comboBusyWait, TRUE);
        gtk_widget_set_sensitive(editThreshold, TRUE);
        gtk_widget_set_sensitive(btnSetThresh, TRUE);
        update_status("FreeRTOS Mode Selected");
    }
}

void on_radio_baremetal_toggled(GtkWidget *widget, gpointer data) {
    (void)data;
    
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
        gtk_widget_set_sensitive(btnConnect, TRUE);
        gtk_widget_set_sensitive(comboBusyWait, FALSE);
        gtk_widget_set_sensitive(editThreshold, FALSE);
        gtk_widget_set_sensitive(btnSetThresh, FALSE);
        update_status("BareMetal Mode Selected");
    }
}

void on_apply_range_clicked(GtkWidget *widget, gpointer data) {
    (void)widget;
    (void)data;
    
    const char* valueStr = gtk_entry_get_text(GTK_ENTRY(editRange));
    int value = atoi(valueStr);
    
    if (value < 5) {
        update_status("Minimum range is 5 cm");
        gtk_entry_set_text(GTK_ENTRY(editRange), "5");
        value = 5;
    } else if (value > 1000) {
        update_status("Maximum range is 1000 cm");
        gtk_entry_set_text(GTK_ENTRY(editRange), "1000");
        value = 1000;
    } else {
        char status[64];
        snprintf(status, sizeof(status), "Graph range: 0 - %d cm", value);
        update_status(status);
    }
    
    graphMaxRange = value;
}

void on_busywait_changed(GtkWidget *widget, gpointer data) {
    (void)data;
    
    int idx = gtk_combo_box_get_active(GTK_COMBO_BOX(widget));
    
    if (idx == 0) {
        busyWaitEnabled = false;
        update_status("Busy-Wait: Inactive");
    } else {
        busyWaitEnabled = true;
        delayBufferHead = 0;
        delayBufferCount = 0;
        update_status("Busy-Wait: Active");
    }
}

void on_reset_stats_clicked(GtkWidget *widget, gpointer data) {
    (void)widget;
    (void)data;
    
    pthread_mutex_lock(&dataMutex);
    freertosLatencySum = 0;
    freertosLatencyCount = 0;
    freertosLatencyMin = 0xFFFFFFFF;
    freertosLatencyMax = 0;
    baremetalLatencySum = 0;
    baremetalLatencyCount = 0;
    baremetalLatencyMin = 0xFFFFFFFF;
    baremetalLatencyMax = 0;
    dataCount = 0;
    dataHead = 0;
    pthread_mutex_unlock(&dataMutex);
    
    update_status("Statistics Reset");
    WriteLog("INFO", "Statistics reset by user");
}

void on_log_toggled(GtkWidget *widget, gpointer data) {
    (void)data;
    
    logEnabled = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget));
    if (logEnabled) {
        update_status("Logging Enabled");
    } else {
        update_status("Logging Disabled");
    }
}

void on_ack_alert_clicked(GtkWidget *widget, gpointer data) {
    (void)widget;
    (void)data;
    
    if (serialFd >= 0) {
        char cmd = 'A';
        write(serialFd, &cmd, 1);
        update_status("Alert Acknowledged");
    }
}

void on_set_threshold_clicked(GtkWidget *widget, gpointer data) {
    (void)widget;
    (void)data;
    
    if (serialFd >= 0) {
        const char* text = gtk_entry_get_text(GTK_ENTRY(editThreshold));
        int value = atoi(text);
        
        char cmd;
        if (value <= 5) { cmd = 'T'; value = 5; }
        else if (value <= 10) { cmd = 'U'; value = 10; }
        else if (value <= 15) { cmd = 'V'; value = 15; }
        else if (value <= 20) { cmd = 'W'; value = 20; }
        else if (value <= 25) { cmd = 'X'; value = 25; }
        else { cmd = 'Y'; value = 30; }
        
        write(serialFd, &cmd, 1);
        
        char status[64];
        snprintf(status, sizeof(status), "Threshold set: %d cm", value);
        update_status(status);
        
        char valueText[8];
        snprintf(valueText, sizeof(valueText), "%d", value);
        gtk_entry_set_text(GTK_ENTRY(editThreshold), valueText);
    }
}

void on_destroy(GtkWidget *widget, gpointer data) {
    (void)widget;
    (void)data;
    
    bRunning = false;
    if (serialFd >= 0) {
        pthread_join(readThread, NULL);
    }
    CloseSerialPort();
    CloseLogFile();
    gtk_main_quit();
}

// ============================================================================
// SERIAL PORT ENUMERATION
// ============================================================================

void populate_serial_ports(GtkWidget *combo) {
    DIR *dir;
    struct dirent *entry;
    
    // Add common serial ports
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo), "ttyUSB0");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo), "ttyUSB1");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo), "ttyACM0");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo), "ttyACM1");
    
    // Scan /dev for additional ttyUSB and ttyACM devices
    dir = opendir("/dev");
    if (dir) {
        while ((entry = readdir(dir)) != NULL) {
            if (strncmp(entry->d_name, "ttyUSB", 6) == 0 ||
                strncmp(entry->d_name, "ttyACM", 6) == 0) {
                // Check if not already added
                gboolean found = FALSE;
                const char* existing[] = {"ttyUSB0", "ttyUSB1", "ttyACM0", "ttyACM1"};
                for (int i = 0; i < 4; i++) {
                    if (strcmp(entry->d_name, existing[i]) == 0) {
                        found = TRUE;
                        break;
                    }
                }
                if (!found) {
                    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo), entry->d_name);
                }
            }
        }
        closedir(dir);
    }
    
    gtk_combo_box_set_active(GTK_COMBO_BOX(combo), 0);
}

// ============================================================================
// MAIN - GTK APPLICATION
// ============================================================================

int main(int argc, char *argv[]) {
    gtk_init(&argc, &argv);
    
    InitLogFile();
    WriteLog("INFO", "Application started (Linux PREEMPT_RT)");
    
    // Main window
    mainWindow = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(mainWindow), "STM32 Real-Time Sensor Monitor (Linux)");
    gtk_window_set_default_size(GTK_WINDOW(mainWindow), 920, 580);
    gtk_window_set_resizable(GTK_WINDOW(mainWindow), FALSE);
    g_signal_connect(mainWindow, "destroy", G_CALLBACK(on_destroy), NULL);
    
    // Main horizontal box
    GtkWidget *mainBox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_container_add(GTK_CONTAINER(mainWindow), mainBox);
    
    // Left panel (controls)
    GtkWidget *leftPanel = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_widget_set_size_request(leftPanel, 230, -1);
    gtk_box_pack_start(GTK_BOX(mainBox), leftPanel, FALSE, FALSE, 5);
    
    // Right panel (graphs)
    GtkWidget *rightPanel = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_box_pack_start(GTK_BOX(mainBox), rightPanel, TRUE, TRUE, 5);
    
    // ===== PORT SETUP SECTION =====
    GtkWidget *portFrame = gtk_frame_new("Port Setup");
    gtk_box_pack_start(GTK_BOX(leftPanel), portFrame, FALSE, FALSE, 2);
    
    GtkWidget *portGrid = gtk_grid_new();
    gtk_container_add(GTK_CONTAINER(portFrame), portGrid);
    gtk_grid_set_row_spacing(GTK_GRID(portGrid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(portGrid), 5);
    gtk_container_set_border_width(GTK_CONTAINER(portGrid), 5);
    
    gtk_grid_attach(GTK_GRID(portGrid), gtk_label_new("Port"), 0, 0, 1, 1);
    comboPort = gtk_combo_box_text_new();
    populate_serial_ports(comboPort);
    gtk_grid_attach(GTK_GRID(portGrid), comboPort, 1, 0, 1, 1);
    
    gtk_grid_attach(GTK_GRID(portGrid), gtk_label_new("Baud"), 0, 1, 1, 1);
    comboBaud = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(comboBaud), "9600");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(comboBaud), "115200");
    gtk_combo_box_set_active(GTK_COMBO_BOX(comboBaud), 1);
    gtk_grid_attach(GTK_GRID(portGrid), comboBaud, 1, 1, 1, 1);
    
    GtkWidget *btnBox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_grid_attach(GTK_GRID(portGrid), btnBox, 0, 2, 2, 1);
    
    btnConnect = gtk_button_new_with_label("Connect");
    gtk_widget_set_sensitive(btnConnect, FALSE);
    g_signal_connect(btnConnect, "clicked", G_CALLBACK(on_connect_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(btnBox), btnConnect, TRUE, TRUE, 0);
    
    btnDisconnect = gtk_button_new_with_label("Disconnect");
    gtk_widget_set_sensitive(btnDisconnect, FALSE);
    g_signal_connect(btnDisconnect, "clicked", G_CALLBACK(on_disconnect_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(btnBox), btnDisconnect, TRUE, TRUE, 0);
    
    // ===== RUN MODE SECTION =====
    GtkWidget *modeFrame = gtk_frame_new("Run Mode");
    gtk_box_pack_start(GTK_BOX(leftPanel), modeFrame, FALSE, FALSE, 2);
    
    GtkWidget *modeBox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_container_add(GTK_CONTAINER(modeFrame), modeBox);
    gtk_container_set_border_width(GTK_CONTAINER(modeBox), 5);
    
    radioBareMetal = gtk_radio_button_new_with_label(NULL, "BareMetal");
    g_signal_connect(radioBareMetal, "toggled", G_CALLBACK(on_radio_baremetal_toggled), NULL);
    gtk_box_pack_start(GTK_BOX(modeBox), radioBareMetal, TRUE, TRUE, 0);
    
    radioFreeRTOS = gtk_radio_button_new_with_label_from_widget(
        GTK_RADIO_BUTTON(radioBareMetal), "FreeRTOS");
    g_signal_connect(radioFreeRTOS, "toggled", G_CALLBACK(on_radio_freertos_toggled), NULL);
    gtk_box_pack_start(GTK_BOX(modeBox), radioFreeRTOS, TRUE, TRUE, 0);
    
    // ===== GRAPH RANGE SECTION =====
    GtkWidget *rangeFrame = gtk_frame_new("Graph Range");
    gtk_box_pack_start(GTK_BOX(leftPanel), rangeFrame, FALSE, FALSE, 2);
    
    GtkWidget *rangeBox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_container_add(GTK_CONTAINER(rangeFrame), rangeBox);
    gtk_container_set_border_width(GTK_CONTAINER(rangeBox), 5);
    
    gtk_box_pack_start(GTK_BOX(rangeBox), gtk_label_new("0 -"), FALSE, FALSE, 0);
    editRange = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(editRange), "20");
    gtk_entry_set_width_chars(GTK_ENTRY(editRange), 5);
    gtk_box_pack_start(GTK_BOX(rangeBox), editRange, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(rangeBox), gtk_label_new("cm"), FALSE, FALSE, 0);
    
    btnApplyRange = gtk_button_new_with_label("Apply");
    g_signal_connect(btnApplyRange, "clicked", G_CALLBACK(on_apply_range_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(rangeBox), btnApplyRange, FALSE, FALSE, 0);
    
    // ===== SCHEDULING OVERHEAD SECTION =====
    GtkWidget *schedFrame = gtk_frame_new("Scheduling Overhead");
    gtk_box_pack_start(GTK_BOX(leftPanel), schedFrame, FALSE, FALSE, 2);
    
    GtkWidget *schedBox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_container_add(GTK_CONTAINER(schedFrame), schedBox);
    gtk_container_set_border_width(GTK_CONTAINER(schedBox), 5);
    
    gtk_box_pack_start(GTK_BOX(schedBox), gtk_label_new("Busy-Wait:"), FALSE, FALSE, 0);
    comboBusyWait = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(comboBusyWait), "Inactive");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(comboBusyWait), "Active");
    gtk_combo_box_set_active(GTK_COMBO_BOX(comboBusyWait), 0);
    g_signal_connect(comboBusyWait, "changed", G_CALLBACK(on_busywait_changed), NULL);
    gtk_box_pack_start(GTK_BOX(schedBox), comboBusyWait, TRUE, TRUE, 0);
    
    // ===== CRITICAL ALERT SECTION =====
    GtkWidget *alertFrame = gtk_frame_new("Critical Alert");
    gtk_box_pack_start(GTK_BOX(leftPanel), alertFrame, FALSE, FALSE, 2);
    
    GtkWidget *alertBox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_add(GTK_CONTAINER(alertFrame), alertBox);
    gtk_container_set_border_width(GTK_CONTAINER(alertBox), 5);
    
    GtkWidget *threshBox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(alertBox), threshBox, FALSE, FALSE, 0);
    
    gtk_box_pack_start(GTK_BOX(threshBox), gtk_label_new("Threshold:"), FALSE, FALSE, 0);
    editThreshold = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(editThreshold), "10");
    gtk_entry_set_width_chars(GTK_ENTRY(editThreshold), 4);
    gtk_box_pack_start(GTK_BOX(threshBox), editThreshold, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(threshBox), gtk_label_new("cm"), FALSE, FALSE, 0);
    
    btnSetThresh = gtk_button_new_with_label("Set");
    g_signal_connect(btnSetThresh, "clicked", G_CALLBACK(on_set_threshold_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(threshBox), btnSetThresh, FALSE, FALSE, 0);
    
    alertStatus = gtk_label_new("Status: OK");
    gtk_label_set_xalign(GTK_LABEL(alertStatus), 0);
    gtk_box_pack_start(GTK_BOX(alertBox), alertStatus, FALSE, FALSE, 0);
    
    btnAckAlert = gtk_button_new_with_label("Acknowledge");
    gtk_widget_set_sensitive(btnAckAlert, FALSE);
    g_signal_connect(btnAckAlert, "clicked", G_CALLBACK(on_ack_alert_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(alertBox), btnAckAlert, FALSE, FALSE, 0);
    
    // ===== OPTIONS SECTION =====
    GtkWidget *optFrame = gtk_frame_new("Options");
    gtk_box_pack_start(GTK_BOX(leftPanel), optFrame, FALSE, FALSE, 2);
    
    GtkWidget *optBox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_add(GTK_CONTAINER(optFrame), optBox);
    gtk_container_set_border_width(GTK_CONTAINER(optBox), 5);
    
    checkLog = gtk_check_button_new_with_label("Enable Logging");
    g_signal_connect(checkLog, "toggled", G_CALLBACK(on_log_toggled), NULL);
    gtk_box_pack_start(GTK_BOX(optBox), checkLog, FALSE, FALSE, 0);
    
    btnResetStats = gtk_button_new_with_label("Reset Stats");
    g_signal_connect(btnResetStats, "clicked", G_CALLBACK(on_reset_stats_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(optBox), btnResetStats, FALSE, FALSE, 0);
    
    // ===== STATUS BAR =====
    statusBar = gtk_label_new("Select Run Mode");
    gtk_label_set_xalign(GTK_LABEL(statusBar), 0);
    GtkWidget *statusFrame = gtk_frame_new(NULL);
    gtk_frame_set_shadow_type(GTK_FRAME(statusFrame), GTK_SHADOW_IN);
    gtk_container_add(GTK_CONTAINER(statusFrame), statusBar);
    gtk_box_pack_end(GTK_BOX(leftPanel), statusFrame, FALSE, FALSE, 2);
    
    // ===== DRAWING AREA =====
    drawingArea = gtk_drawing_area_new();
    gtk_widget_set_size_request(drawingArea, 650, 500);
    g_signal_connect(drawingArea, "draw", G_CALLBACK(on_draw), NULL);
    gtk_box_pack_start(GTK_BOX(rightPanel), drawingArea, TRUE, TRUE, 5);
    
    // Disable FreeRTOS-only controls initially
    gtk_widget_set_sensitive(comboBusyWait, FALSE);
    gtk_widget_set_sensitive(editThreshold, FALSE);
    gtk_widget_set_sensitive(btnSetThresh, FALSE);
    
    // Setup UI update timer (100ms)
    g_timeout_add(GRAPH_UPDATE_MS, update_ui, NULL);
    
    gtk_widget_show_all(mainWindow);
    gtk_main();
    
    return 0;
}
