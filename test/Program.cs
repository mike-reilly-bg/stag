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
            double[] result, 
            int resultLen,
            int errorCorrection
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
                double[] _result = new double[23];

                FindStagCorners(pixels, w, h, _result, _result.Length, 30);

                int num = (int)_result[0];
                Console.WriteLine($"Found {num} marker(s)");

                for (int i = 0; i < num && i < 2; i++)
                {
                    Console.Write($"Marker {i}: ");
                    for (int k = 0; k < 4; k++)
                    {
                        double x = _result[1 + i * 8 + k * 2];
                        double y = _result[1 + i * 8 + k * 2 + 1];
                        Console.Write($"({x:F1},{y:F1}) ");
                    }
                    Console.WriteLine();
                }
                for (int i = 1; i < 7; i++)
                {
                    int ind = 16 + i;
                    Console.Write($"({ind},{_result[16+i]:F1}");
                    Console.WriteLine();
                }
            }
        }
    }
}
