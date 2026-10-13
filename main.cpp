
#include <stdio.h>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <random>
#include <iomanip>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>
#include <arpa/inet.h>
#include "OwnshipSmoother.h"

#define WIDTH  800
#define HEIGHT 800

// NOTE: Uncomment the following line for GL error handling
//#define GL_DEBUG

#ifdef GL_DEBUG
#define GLCALL(function) \
   { \
      GLenum error = GL_INVALID_ENUM; \
      while (error != GL_NO_ERROR) \
      { \
         error = glGetError(); \
      } \
      function; \
      error = glGetError(); \
      if (error != GL_NO_ERROR) \
      { \
         fprintf(stderr, "OpenGL Error: GL_ENUM(%d) at %s:%d\n", error, __FILE__, __LINE__); \
      } \
   }
#else
#define GLCALL(function) function;
#endif

// Send packet start marker
// 00 40 00 20
const uint32_t tx_start_marker = 0x20004000;

// Receive packet start marker
// 0e 10 00 00
// 0e 30 00 00
const uint32_t rx_start_marker = 0x0000100e;
const uint32_t alt_rx_start_marker = 0x0000300e;

// End packet marker
// 13 00 .. .. (last two bytes are packet number)
const uint32_t end_marker = 0xffff0013;

struct Timestamp
{
   uint32_t Seconds;
   uint32_t Microseconds;
};

struct TOwnshipFrame
{
   double    Latitude;
   double    Longitude;
   float     Altitude;
   float     Heading;
   float     Pitch;
   float     Roll;

   double    DeltaLatitude;
   double    DeltaLongitude;
   float     DeltaAltitude;
   float     DeltaHeading;
   float     DeltaPitch;
   float     DeltaRoll;

   Timestamp TimestampValue;
   double    DeltaMs;
};

struct Ownship
{
   uint32_t Packet;
   std::vector<TOwnshipFrame> Frames;
};

struct MotionDeviceFrame
{
   uint16_t PacketNumTx;
   uint16_t PacketNumRx;
   uint16_t MotionRequestId[2];
   uint16_t MotionResponseId[2];
   Timestamp TimestampTx;
   Timestamp TimestampRx;
   double DeltaTxMs;
   double DeltaRxMs;
};

struct MotionDevice
{
   uint32_t NumPacketsTx;
   uint32_t NumPacketsRx;
   uint32_t MaxPacketSizeTx;
   uint32_t MinPacketSizeTx;
   uint32_t MaxPacketSizeRx;
   uint32_t MinPacketSizeRx;
   uint32_t TotalMotionRequests[2];
   uint32_t TotalMotionResponses[2];
   std::vector<MotionDeviceFrame> Frames;
};

struct MotionDeviceControl
{
   uint8_t  PacketType;
   uint8_t  Words;
   uint16_t RequestId;
   uint8_t  SensorId;
   uint8_t  DeviceId;
   uint16_t Pad;
};

struct MotionDeviceReturn
{
   uint8_t  PacketType;
   uint8_t  Words;
   uint16_t RequestId;
   uint8_t  SensorId;
   uint8_t  DeviceId;
   uint16_t Pad;
};

MotionDevice motion_device = {};
Ownship ownship = {};

uint64_t swap48(uint64_t value)
{
    value &= 0x0000FFFFFFFFFFFFULL;

    return ((value & 0x0000000000FFULL) << 40) |
           ((value & 0x00000000FF00ULL) << 24) |
           ((value & 0x000000FF0000ULL) <<  8) |
           ((value & 0x0000FF000000ULL) >>  8) |
           ((value & 0x00FF00000000ULL) >> 24) |
           ((value & 0xFF0000000000ULL) >> 40);
}

double decode48SB1ToDegrees(char* MsbBuffer, char* LsbBuffer)
{
   int32_t raw_msb = *(int32_t*)MsbBuffer;
   int16_t raw_lsb = *(int16_t*)LsbBuffer;

   int64_t raw = swap48(raw_lsb * 65536 + raw_msb);

   if (raw >= 140737488355328)
      raw = raw - 281474976710656;

   double revolutions = raw / 140737488355328.0;
   double degrees = revolutions * 180.0;

   if (degrees < -180.0)
      degrees += 360.0;

   if (degrees > 180.0)
      degrees -= 360.0;

   return degrees;
}

float decode32SB1ToDegrees(char* Buffer)
{
   int32_t raw = htonl(*(int32_t*)Buffer);
   float revolutions = raw / 2147483648.0;
   float degrees = revolutions * 180.0;

   if (degrees < -180.0)
      degrees += 360.0;

   if (degrees > 180.0)
      degrees -= 360.0;

   return degrees;
}

float decode32SB23ToFeet(char* Buffer)
{
   int32_t raw = htonl(*(int32_t*)Buffer);
   float altitude_feet = raw / 256.0;
   return altitude_feet;
}

int ReadCaptureMotionReturn(const char* Filename)
{
   char* buffer = nullptr;
   int bytes = 0;
   FILE* fd = fopen(Filename, "rb");

   if (fd)
   {
      if (fseek(fd, 0, SEEK_END) != 0)
      {
         fclose(fd);
         return 1;
      }

      bytes = ftell(fd);

      fseek(fd, 0, SEEK_SET);

      buffer = new char[bytes];
      memset(buffer, 0, bytes);

      fread(buffer, bytes, 1, fd);

      fclose(fd);
   }

   // Analyze the buffer
   bool processing_send_packet = false;
   int  count = 0;
   int  packet_start = 0;
   MotionDeviceFrame frame;
   for (int i = 0; i < bytes; i += 4)
   {
      if (*((uint32_t*)&buffer[i]) == tx_start_marker)
      {
         processing_send_packet = true;
         memset(&frame, 0, sizeof(frame));
         packet_start = i;
         motion_device.NumPacketsTx++;
         frame.TimestampTx = *(Timestamp*)&buffer[i-60];
      }
      else if (*((uint32_t*)&buffer[i]) == rx_start_marker ||
               *((uint32_t*)&buffer[i]) == alt_rx_start_marker)
      {
         processing_send_packet = false;
         packet_start = i;
         motion_device.NumPacketsRx++;
         frame.TimestampRx = *(Timestamp*)&buffer[i-60];
      }
      else if (buffer[i] == 0x13 && buffer[i+1] == 0x00)
      {
         {
            int size = i - packet_start + 4;
            if (processing_send_packet)
            {
               if (motion_device.MaxPacketSizeTx == 0 || size > motion_device.MaxPacketSizeTx)
               {
                  motion_device.MaxPacketSizeTx = size;
               }
               if (motion_device.MinPacketSizeTx == 0 || size < motion_device.MinPacketSizeTx)
               {
                  motion_device.MinPacketSizeTx = size;
               }
               frame.PacketNumTx = ntohs(*(uint16_t*)&buffer[i+2]);
            }
            else
            {
               if (motion_device.MaxPacketSizeRx == 0 || size > motion_device.MaxPacketSizeRx)
               {
                  motion_device.MaxPacketSizeRx = size;
               }
               if (motion_device.MinPacketSizeRx == 0 || size < motion_device.MinPacketSizeRx)
               {
                  motion_device.MinPacketSizeRx = size;
               }
               frame.PacketNumRx = ntohs(*(uint16_t*)&buffer[i+2]);
               motion_device.Frames.push_back(frame);
            }
         }
      }
      else if (processing_send_packet)
      {
         MotionDeviceControl* mdc = (MotionDeviceControl*)&buffer[i];
         if (mdc->PacketType == 0x40 && mdc->Words == 1 && mdc->SensorId == 1 && mdc->DeviceId == 1)
         {
            frame.MotionRequestId[0] = ntohs(mdc->RequestId);
            motion_device.TotalMotionRequests[0]++;
         }
         else if (mdc->PacketType == 0x40 && mdc->Words == 1 && mdc->SensorId == 2 && mdc->DeviceId == 2)
         {
            frame.MotionRequestId[1] = ntohs(mdc->RequestId);
            motion_device.TotalMotionRequests[1]++;
         }
      }
      else
      {
         MotionDeviceReturn* mdr = (MotionDeviceReturn*)&buffer[i];
         if (mdr->PacketType == 0x41 && mdr->Words == 13 && mdr->SensorId & 0x01 && mdr->DeviceId & 0x01)
         {
            frame.MotionResponseId[0] = ntohs(mdr->RequestId);
            motion_device.TotalMotionResponses[0]++;
         }
         else if (mdr->PacketType == 0x41 && mdr->Words == 13 && mdr->SensorId & 0x02 && mdr->DeviceId & 0x02)
         {
            frame.MotionResponseId[1] = ntohs(mdr->RequestId);
            motion_device.TotalMotionResponses[1]++;
         }
      }
   }

   delete [] buffer;

   // Write out csv file
   std::string out_filename = Filename + std::string(".csv");
   FILE* out_fd = fopen(out_filename.c_str(), "w");

   if (out_fd)
   {
      fprintf(out_fd, "Frame,Timestamp (Tx),Timestamp (Rx),Packet Number (Tx),Packet Number (Rx),Device 1 Request ID,Device 2 Request ID,Device 1 Response ID,Device 2 Response ID\n");
      int idx = 0;
      for (auto& frame : motion_device.Frames)
      {
         fprintf(out_fd, "%d,%d.%06d,%d.%06d,%d,%d,%d,%d,%d,%d\n",
            idx + 1,
            frame.TimestampTx.Seconds,
            frame.TimestampTx.Microseconds,
            frame.TimestampRx.Seconds,
            frame.TimestampRx.Microseconds,
            frame.PacketNumTx,
            frame.PacketNumRx,
            frame.MotionRequestId[0],
            frame.MotionRequestId[1],
            frame.MotionResponseId[0],
            frame.MotionResponseId[1]);

         if (idx > 0)
         {
            const auto& pv_frame = motion_device.Frames[idx-1];
            if (frame.TimestampTx.Microseconds > pv_frame.TimestampTx.Microseconds)
            {
               frame.DeltaTxMs = (double)(frame.TimestampTx.Microseconds - pv_frame.TimestampTx.Microseconds) / 1000.0;
            }
            else
            {
               frame.DeltaTxMs = (double)(frame.TimestampTx.Microseconds - pv_frame.TimestampTx.Microseconds + 1000000) / 1000.0;
            }
            if (frame.TimestampRx.Microseconds > pv_frame.TimestampRx.Microseconds)
            {
               frame.DeltaRxMs = (double)(frame.TimestampRx.Microseconds - pv_frame.TimestampRx.Microseconds) / 1000.0;
            }
            else
            {
               frame.DeltaRxMs = (double)(frame.TimestampRx.Microseconds - pv_frame.TimestampRx.Microseconds + 1000000) / 1000.0;
            }
         }

         idx++;
      }
      fclose(out_fd);
   }

   return bytes;
}

int ReadCaptureOwnship(const char* Filename)
{
   char* buffer = nullptr;
   int bytes = 0;
   FILE* fd = fopen(Filename, "rb");

   if (fd)
   {
      if (fseek(fd, 0, SEEK_END) != 0)
      {
         fclose(fd);
         return 1;
      }

      bytes = ftell(fd);

      fseek(fd, 0, SEEK_SET);

      buffer = new char[bytes];
      memset(buffer, 0, bytes);

      fread(buffer, bytes, 1, fd);

      fclose(fd);
   }

   // Analyze the buffer
   bool processing_packet = false;
   int  count = 0;
   int  packet_start = 0;
   TOwnshipFrame frame;
   for (int i = 0; i < bytes;)
   {
      if (!processing_packet)
      {
         if (*((uint32_t*)&buffer[i]) == tx_start_marker)
         {
            memset(&frame, 0, sizeof(frame));
            packet_start = i;
            frame.TimestampValue = *(Timestamp*)&buffer[i-58];
            processing_packet = true;

            frame.Roll = decode32SB1ToDegrees(&buffer[i+24]);
            frame.Pitch = decode32SB1ToDegrees(&buffer[i+28]);
            frame.Heading = decode32SB1ToDegrees(&buffer[i+32]);
            frame.Altitude = decode32SB23ToFeet(&buffer[i+48]);
            frame.Latitude = decode48SB1ToDegrees(&buffer[i+36], &buffer[i+40]);
            frame.Longitude = decode48SB1ToDegrees(&buffer[i+44], &buffer[i+42]);
         }
         else
         {
            i++;
         }
      }
      else
      {
         if (buffer[i] == 0x13 && buffer[i+1] == 0x00)
         {
            processing_packet = false;
            count++;
            ownship.Packet++;
            if (ownship.Frames.size() > 0)
            {
               int idx = ownship.Frames.size() - 1;
               TOwnshipFrame& pv_frame = ownship.Frames[idx];

               if (frame.TimestampValue.Microseconds > pv_frame.TimestampValue.Microseconds)
               {
                  frame.DeltaMs = (double)(frame.TimestampValue.Microseconds - pv_frame.TimestampValue.Microseconds) / 1000000.0;
               }
               else
               {
                  frame.DeltaMs = (double)(frame.TimestampValue.Microseconds - pv_frame.TimestampValue.Microseconds + 1000000) / 1000000.0;
               }

               frame.DeltaLatitude = frame.Latitude - pv_frame.Latitude;
               frame.DeltaLongitude = frame.Longitude - pv_frame.Longitude;
               frame.DeltaAltitude = frame.Altitude - pv_frame.Altitude;
               frame.DeltaHeading = frame.Heading - pv_frame.Heading;
               frame.DeltaPitch = frame.Pitch - pv_frame.Pitch;
               frame.DeltaRoll = frame.Roll - pv_frame.Roll;
            }

            ownship.Frames.push_back(frame);
         }
         else
         {
            i += 4;
         }
      }
   }

   delete [] buffer;

   return bytes;
}

int main(int argc, char* argv[])
{
   GLFWwindow* window = nullptr;

   if (argc != 3)
   {
      printf("Usage: main <filename> [motion,ownship]\n");
      return 0;
   }

   bool read_motion = (strcmp(argv[2], "motion") == 0);

   const char* filename = argv[1];

   int file_size = 0;
   
   if (read_motion)
      file_size = ReadCaptureMotionReturn(filename);
   else
      file_size = ReadCaptureOwnship(filename);

   if (file_size <= 0)
   {
      printf("Error reading %s\n", filename);
      return 0;
   }

   // initialize glfw
   if (!glfwInit())
      return 0;

   glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
   glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
   glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

   // Create window
   window = glfwCreateWindow(WIDTH, HEIGHT, "Vital Decoder", NULL, NULL);

   if (!window)
   {
      glfwTerminate();
      return 0;
   }

   // make the window's context current
   glfwMakeContextCurrent(window);

   // use glad to load OpenGL function pointers
   if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
   {
      printf("Error: Failed to initialize GLAD.\n");
      glfwTerminate();
      return 0;
   }

   glfwSetWindowSize(window, WIDTH, HEIGHT);

   // Setup Dear ImGui
   IMGUI_CHECKVERSION();
   ImGui::CreateContext();
   ImPlot::CreateContext();
   ImGui::StyleColorsDark();

   // Setup Platform/Render backends
   ImGui_ImplGlfw_InitForOpenGL(window, true);
   ImGui_ImplOpenGL3_Init("#version 330");

   // Make the window visible
   glfwShowWindow(window);

   // Initialize opengl
   GLCALL(glClearColor(0.5, 0.5, 0.5, 1.0));

   // enable blending
   GLCALL(glEnable(GL_BLEND));
   GLCALL(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

   // set frame rate to 60 Hz
   using framerate = std::chrono::duration<double, std::ratio<1, 60>>;
   auto frame_time = std::chrono::high_resolution_clock::now() + framerate{1};

   float* plot_time = nullptr;
   float* time_delta = nullptr;
   double* plot_time_d = nullptr;
   double* lat = nullptr;
   double* lat_delta = nullptr;
   double* lon = nullptr;
   double* lon_delta = nullptr;
   float* alt = nullptr;
   float* alt_delta = nullptr;
   float* heading = nullptr;
   float* heading_delta = nullptr;
   float* pitch = nullptr;
   float* pitch_delta = nullptr;
   float* roll = nullptr;
   float* roll_delta = nullptr;
   int samples = ownship.Frames.size();
   int offset = 0;

   bool smooth = false;
   OwnshipSmoother smoother;

   if (samples > 0)
   {
      if (smooth)
      {
         plot_time = new float[samples];
         time_delta = new float[samples];
         plot_time_d = new double[samples];
         lat = new double[samples];
         lat_delta = new double[samples];
         lon = new double[samples];
         lon_delta = new double[samples];
         alt = new float[samples];
         alt_delta = new float[samples];
         heading = new float[samples];
         heading_delta = new float[samples];
         pitch = new float[samples];
         pitch_delta = new float[samples];
         roll = new float[samples];
         roll_delta = new float[samples];

         memset(plot_time, 0, sizeof(float) * (samples));
         memset(time_delta, 0, sizeof(float) * (samples));
         memset(plot_time_d, 0, sizeof(double) * (samples));
         memset(lat, 0, sizeof(double) * (samples));
         memset(lat_delta, 0, sizeof(double) * (samples));
         memset(lon, 0, sizeof(double) * (samples));
         memset(lon_delta, 0, sizeof(double) * (samples));
         memset(lon, 0, sizeof(float) * (samples));
         memset(lon_delta, 0, sizeof(float) * (samples));
         memset(heading, 0, sizeof(float) * (samples));
         memset(heading_delta, 0, sizeof(float) * (samples));
         memset(pitch, 0, sizeof(float) * (samples));
         memset(pitch_delta, 0, sizeof(float) * (samples));
         memset(roll, 0, sizeof(float) * (samples));
         memset(roll_delta, 0, sizeof(float) * (samples));

         for (int i = 0; i < samples; i++)
         {
            plot_time[i] += ownship.Frames[i].DeltaMs;
            plot_time_d[i] += ownship.Frames[i].DeltaMs;

            OwnshipFrame frame;

            frame.Latitude = ownship.Frames[i].Latitude;
            frame.Longitude = ownship.Frames[i].Longitude;
            frame.Altitude = ownship.Frames[i].Altitude;
            frame.Yaw = ownship.Frames[i].Heading;
            frame.Pitch = ownship.Frames[i].Pitch;
            frame.Roll = ownship.Frames[i].Roll;

            smoother.AddFrame(frame);

            if (i > 0)
            {
               plot_time[i] += plot_time[i-1];
               plot_time_d[i] += plot_time[i-1];
            }

            time_delta[i] = ownship.Frames[i].DeltaMs;

            if (smoother.GetOutput(frame))
            {
               lat[i] = frame.Latitude;
               lon[i] = frame.Longitude;
               alt[i] = frame.Altitude;
               heading[i] = frame.Yaw;
               pitch[i] = frame.Pitch;
               roll[i] = frame.Roll;

               if (i > 0)
               {
                  lat_delta[i] = lat[i] - lat[i-1];
                  lon_delta[i] = lon[i] - lon[i-1];
                  alt_delta[i] = alt[i] - alt[i-1];
                  heading_delta[i] = heading[i] - heading[i-1];
                  pitch_delta[i] = pitch[i] - pitch[i-1];
                  roll_delta[i] = roll[i] - roll[i-1];
               }
            }
            else
            {
               lat[i] = 0.0;
               lat_delta[i] = 0.0;
               lon[i] = 0.0;
               lon_delta[i] = 0.0;
               alt[i] = 0.0;
               alt_delta[i] = 0.0;
               heading[i] = 0.0;
               heading_delta[i] = 0.0;
               pitch[i] = 0.0;
               pitch_delta[i] = 0.0;
               roll[i] = 0.0;
               roll_delta[i] = 0.0;
            }
         }
      }
      else
      {
         plot_time = new float[samples-1];
         time_delta = new float[samples-1];
         plot_time_d = new double[samples-1];
         lat = new double[samples-1];
         lat_delta = new double[samples-1];
         lon = new double[samples-1];
         lon_delta = new double[samples-1];
         alt = new float[samples-1];
         alt_delta = new float[samples-1];
         heading = new float[samples-1];
         heading_delta = new float[samples-1];
         pitch = new float[samples-1];
         pitch_delta = new float[samples-1];
         roll = new float[samples-1];
         roll_delta = new float[samples-1];

         memset(plot_time, 0, sizeof(float) * (samples-1));
         memset(time_delta, 0, sizeof(float) * (samples-1));
         memset(plot_time_d, 0, sizeof(double) * (samples-1));
         memset(lat, 0, sizeof(double) * (samples-1));
         memset(lat_delta, 0, sizeof(double) * (samples-1));
         memset(lon, 0, sizeof(double) * (samples-1));
         memset(lon_delta, 0, sizeof(double) * (samples-1));
         memset(lon, 0, sizeof(float) * (samples-1));
         memset(lon_delta, 0, sizeof(float) * (samples-1));
         memset(heading, 0, sizeof(float) * (samples-1));
         memset(heading_delta, 0, sizeof(float) * (samples-1));
         memset(pitch, 0, sizeof(float) * (samples-1));
         memset(pitch_delta, 0, sizeof(float) * (samples-1));
         memset(roll, 0, sizeof(float) * (samples-1));
         memset(roll_delta, 0, sizeof(float) * (samples-1));

         for (int i = 1; i < samples; i++)
         {
            plot_time[i-1] += ownship.Frames[i].DeltaMs;
            plot_time_d[i-1] += ownship.Frames[i].DeltaMs;

            if (i > 1)
            {
               plot_time[i-1] += plot_time[i-2];
               plot_time_d[i-1] += plot_time[i-2];
            }

            time_delta[i-1] = ownship.Frames[i].DeltaMs;
            lat[i-1] = ownship.Frames[i].Latitude;
            lat_delta[i-1] = ownship.Frames[i].DeltaLatitude;
            lon[i-1] = ownship.Frames[i].Longitude;
            lon_delta[i-1] = ownship.Frames[i].DeltaLongitude;
            alt[i-1] = ownship.Frames[i].Altitude;
            alt_delta[i-1] = ownship.Frames[i].DeltaAltitude;
            heading[i-1] = ownship.Frames[i].Heading;
            heading_delta[i-1] = ownship.Frames[i].DeltaHeading;
            pitch[i-1] = ownship.Frames[i].Pitch;
            pitch_delta[i-1] = ownship.Frames[i].DeltaPitch;
            roll[i-1] = ownship.Frames[i].Roll;
            roll_delta[i-1] = ownship.Frames[i].DeltaRoll;
         }
         samples--;
      }
   }

   while (window)
   {
      // Poll events
      glfwPollEvents();

      if (glfwWindowShouldClose(window))
      {
         glfwTerminate();
         window = nullptr;
         break;
      }

      GLCALL(glClear(GL_COLOR_BUFFER_BIT));

      // Start the Dear ImGui frame
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      if (read_motion)
      {
         ImGui::Begin("Motion Device");

         ImGui::Text("Filename:             %s", filename);
         ImGui::Text("File size:            %d", file_size);
         ImGui::Text("Packets sent:         %d", motion_device.NumPacketsTx);
         ImGui::Text("Packets received:     %d", motion_device.NumPacketsRx);
         ImGui::Text("Max Packet size (Tx): %d", motion_device.MaxPacketSizeTx);
         ImGui::Text("Min Packet size (Tx): %d", motion_device.MinPacketSizeTx);
         ImGui::Text("Max Packet size (Rx): %d", motion_device.MaxPacketSizeRx);
         ImGui::Text("Min Packet size (Rx): %d", motion_device.MinPacketSizeRx);
         ImGui::Text("Frames:               %d", motion_device.Frames.size());
         ImGui::Text("Total Requests #1:    %d", motion_device.TotalMotionRequests[0]);
         ImGui::Text("Total Requests #2:    %d", motion_device.TotalMotionRequests[1]);
         ImGui::Text("Total Responses #1:   %d", motion_device.TotalMotionResponses[0]);
         ImGui::Text("Total Responses #2:   %d", motion_device.TotalMotionResponses[1]);
         ImGui::Separator();

         static int frame = 0;
         int max_frame = motion_device.Frames.size()-1;

         ImGui::SliderInt("##Frame", &frame, 0, max_frame);
         ImGui::SameLine();
         if (ImGui::Button("-"))
         {
            if (frame > 0)
               frame--;
         }
         ImGui::SameLine();
         if (ImGui::Button("+"))
         {
            if (frame < max_frame)
               frame++;
         }

         ImGui::Text("Timestamp Tx:     %d.%06d (delta: %f ms)",
            motion_device.Frames[frame].TimestampTx.Seconds,
            motion_device.Frames[frame].TimestampTx.Microseconds,
            motion_device.Frames[frame].DeltaTxMs);
         ImGui::Text("Timestamp Rx:     %d.%06d (delta: %f ms)",
            motion_device.Frames[frame].TimestampRx.Seconds,
            motion_device.Frames[frame].TimestampRx.Microseconds,
            motion_device.Frames[frame].DeltaRxMs);
         ImGui::Text("Packet Number Tx: %d", motion_device.Frames[frame].PacketNumTx);
         ImGui::Text("Packet Number Rx: %d", motion_device.Frames[frame].PacketNumRx);
         ImGui::Text("Request ID #1:    %d", motion_device.Frames[frame].MotionRequestId[0]);
         ImGui::Text("Request ID #2:    %d", motion_device.Frames[frame].MotionRequestId[1]);
         ImGui::Text("Response ID #1:   %d", motion_device.Frames[frame].MotionResponseId[0]);
         ImGui::Text("Response ID #2:   %d", motion_device.Frames[frame].MotionResponseId[1]);
      }
      else
      {
         ImGui::Begin("Ownship");

         if (ImPlot::BeginPlot("Ownship"))
         {
            ImPlot::SetupAxes("x - Iteration", "y - Ownship");                     
            //ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, 60.0, ImGuiCond_Once);         
            //ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 20.0, ImGuiCond_Once);         

            ImPlot::PlotLine("Time Delta", plot_time, time_delta, samples);
            //ImPlot::PlotLine("Latitude", plot_time_d, lat, samples);
            //ImPlot::PlotLine("Latitude Delta", plot_time_d, lat_delta, samples);
            //ImPlot::PlotLine("Longitude", plot_time_d, lon, samples);
            //ImPlot::PlotLine("Longitude Delta", plot_time_d, lon_delta, samples);
            //ImPlot::PlotLine("Altitude", plot_time, alt, samples);
            //ImPlot::PlotLine("Altitude Delta", plot_time, alt_delta, samples);
            //ImPlot::PlotLine("Heading", plot_time, heading, samples);
            ImPlot::PlotLine("Heading Delta", plot_time, heading_delta, samples);
            //ImPlot::PlotLine("Pitch", plot_time, pitch, samples);
            ImPlot::PlotLine("Pitch Delta", plot_time, pitch_delta, samples);
            //ImPlot::PlotLine("Roll", plot_time, roll, samples);
            ImPlot::PlotLine("Roll Delta", plot_time, roll_delta, samples);
         }
         ImPlot::EndPlot();
      }

      ImGui::End();

      // Render ImGui
      ImGui::Render();
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

      glfwSwapBuffers(window);

      // wait until next frame
      std::this_thread::sleep_until(frame_time);
      frame_time += framerate{1};
   }

   // Cleanup
   ImGui_ImplOpenGL3_Shutdown();
   ImGui_ImplGlfw_Shutdown();
   ImPlot::DestroyContext();
   ImGui::DestroyContext();

   glfwDestroyWindow(window);
   glfwTerminate();

   return 0;
}
