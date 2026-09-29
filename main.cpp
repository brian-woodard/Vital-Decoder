
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
#include <arpa/inet.h>

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

int ReadCapture(const char* Filename)
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

int main(int argc, char* argv[])
{
   GLFWwindow* window = nullptr;

   if (argc != 2)
   {
      printf("Usage: main <filename>\n");
      return 0;
   }

   const char* filename = argv[1];
   int file_size = ReadCapture(filename);

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
   ImGui::DestroyContext();

   glfwDestroyWindow(window);
   glfwTerminate();

   return 0;
}
