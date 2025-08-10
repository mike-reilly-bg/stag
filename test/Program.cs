using System;
using System.Runtime.InteropServices;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace StagTester
{
    class Program
    {
        // P/Invoke into your native wrapper
        [DllImport("stagdll", EntryPoint = "FindStagCorners", CallingConvention = CallingConvention.Cdecl)]
        public static extern void FindStagCorners(
            byte[] image, 
            int width, 
            int height, 
            float[] result, 
            int resultLen
        );

        static void Main(string[] args)
        {
            // image path (you can pass in via args[0] if you like)
            string path = args.Length > 0 ? args[0] : "example.jpg";

            // load as 3‐channel BGR; ImageSharp uses Rgb24, 
            // but OpenCV cvtColor(BGR->GRAY) will still work
            using (Image<Rgb24> img = Image.Load<Rgb24>(path))
            {
                int w = img.Width, h = img.Height;
                byte[] pixels = new byte[w * h * 3];
                img.CopyPixelDataTo(pixels);

                // result[0] = count, then up to 2 markers × 8 floats each
                float[] result = new float[1 + 2 * 4 * 2];

                FindStagCorners(pixels, w, h, result, result.Length);

                int num = (int)result[0];
                Console.WriteLine($"Found {num} marker(s)");

                for (int i = 0; i < num && i < 2; i++)
                {
                    Console.Write($"Marker {i}: ");
                    for (int k = 0; k < 4; k++)
                    {
                        float x = result[1 + i * 8 + k * 2];
                        float y = result[1 + i * 8 + k * 2 + 1];
                        Console.Write($"({x:F1},{y:F1}) ");
                    }
                    Console.WriteLine();
                }
            }
        }
    }
}
