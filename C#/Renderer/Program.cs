using GMath;
using Rendering;
using System;
using static GMath.Gfx;
using System.Collections.Generic;

namespace Renderer
{
    class Program
    {
        static void Main(string[] args)
        {
            Raster render = new Raster(1024, 512);
            MyTransform(render);
            //FreeTransformTest(render);
            //DrawRoomTest(render);
            render.RenderTarget.Save("test.rbm");
            Console.WriteLine("Done.");
        }

        public static float3[] RandomPositionsInSphereSurface(int N)
        {
            float3[] points = new float3[N];

            for (int i = 0; i < N; i++)
                points[i] = randomInSphere();

            return points;
        }

        public static float3[] RandomPositionsInCylinderSurface(int N, float _base)
        {
            float3[] points = new float3[N];

            for (int i = 0; i < N; i++)
                points[i] = randomInCylinder(_base);

            return points;
        }
        public static float3[] RandomPositionsInBoxSurface(int N)
        {
            float3[] points = new float3[N];

            for (int i = 0; i < N; i++)
                points[i] = randomInBox();

            return points;
        }

        public static float3[] RandomPositionsInCircleSurface(int N, int pos_z)
        {
            float3[] points = new float3[N];

            for (int i = 0; i < N; i++)
                points[i] = randomInCircle(pos_z);

            return points;
        }


        public static float3[] ApplyTransform(float3[] points, float4x4 matrix)
        {
            float3[] result = new float3[points.Length];

            // Transform points with a matrix
            // Linear transform in homogeneous coordinates
            for (int i = 0; i < points.Length; i++)
            {
                float4 h = float4(points[i], 1);
                h = mul(h, matrix);
                result[i] = h.xyz / h.w;
            }

            return result;
        }

        public static float3[] ApplyTransform(float3[] points, Func<float3, float3> freeTransform)
        {
            float3[] result = new float3[points.Length];

            // Transform points with a function
            for (int i = 0; i < points.Length; i++)
                result[i] = freeTransform(points[i]);

            return result;
        }

        public static void MyTransform(Raster render)
        {
            render.ClearRT(float4(0, 0, 0.2f, 1));
            int N = 10000;
            float3[] points_sphere = RandomPositionsInSphereSurface(N);

            float3[] points_cylinder1 = RandomPositionsInCylinderSurface(N/4, 0);//base de la lampara
            float3[] points_cylinder2 = RandomPositionsInCylinderSurface(N/10, (float)0.2);//tubo 1
            float3[] points_cylinder3 = RandomPositionsInCylinderSurface(N/10, (float)1.02);//tubo 2
            float3[] points_circle = RandomPositionsInCircleSurface(N/4, 1);//tapa de la base


            //poner la tapa a la base
            List<float3> Cylinder = new List<float3>(points_cylinder1);
            Cylinder.AddRange(points_circle);
            points_cylinder1 = Cylinder.ToArray();


            //llevar a escala la base
            points_cylinder1 = ApplyTransform(points_cylinder1, Transforms.Scale((float)0.8, (float)0.4, (float)0.8));

            //llevar a escala el tubo1
            points_cylinder2 = ApplyTransform(points_cylinder2, Transforms.Scale((float)0.04, (float)2, (float)0.04));

            //llevar a escala rotar y trasladar el tubo2 
            points_cylinder3 = ApplyTransform(points_cylinder3, Transforms.Scale((float)0.04, (float)2, (float)0.04));
            points_cylinder3 = ApplyTransform(points_cylinder3, Transforms.RotateZGrad(-10));
            points_cylinder3 = ApplyTransform(points_cylinder3, Transforms.RotateXGrad(-10));
            points_cylinder3 = ApplyTransform(points_cylinder3, Transforms.Translate((float)0.8, (float)0.6, 0));

            //llevar a escala y ubicar la esfera
            points_sphere = ApplyTransform(points_sphere, Transforms.Scale((float)0.7, (float)0.7, (float)0.7));
            points_sphere = ApplyTransform(points_sphere, Transforms.Translate((float)0.6,(float)1.8,0));


            //poner ambos tubos en la base
            List<float3> Cylinder2 = new List<float3>(points_cylinder1);
            Cylinder2.AddRange(points_cylinder2);
            Cylinder2.AddRange(points_cylinder3);
            points_cylinder1 = Cylinder2.ToArray();

            //ubicar la figura completa(ya la esfera estaba ubicada)
            points_cylinder1 = ApplyTransform(points_cylinder1, Transforms.Translate(0, -2, 0));



            points_sphere = ApplyTransform(points_sphere, Transforms.LookAtLH(float3(5f, 2.6f, 4), float3(0, 0, 0), float3(0, 1, 0)));
            points_sphere = ApplyTransform(points_sphere, Transforms.PerspectiveFovLH(pi_over_4, render.RenderTarget.Height / (float)render.RenderTarget.Width, 0.01f, 10));

            points_cylinder1 = ApplyTransform(points_cylinder1, Transforms.LookAtLH(float3(5f, 2.6f, 4), float3(0, 0, 0), float3(0, 1, 0)));
            points_cylinder1 = ApplyTransform(points_cylinder1, Transforms.PerspectiveFovLH(pi_over_4, render.RenderTarget.Height / (float)render.RenderTarget.Width, 0.01f, 10));



            
            render.DrawPoints(points_cylinder1);
            render.DrawPoints(points_sphere);

        }

        private static void FreeTransformTest(Raster render)
        {
            render.ClearRT(float4(0, 0, 0.2f, 1)); // clear with color dark blue.

            int N = 100000;
            // Create buffer with points to render
            float3[] points = RandomPositionsInBoxSurface(N);

            // Creating boxy...
            points = ApplyTransform(points, float4x4(
                1f, 0, 0, 0,
                0, 1.57f, 0, 0,
                0, 0, 1f, 0,
                0, 0, 0, 1
                ));

            // Apply a free transform
            points = ApplyTransform(points, p => float3(p.x * cos(p.y) + p.z * sin(p.y), p.y, p.x * sin(p.y) - p.z * cos(p.y)));

            #region viewing and projecting

            points = ApplyTransform(points, Transforms.LookAtLH(float3(5f, 2.6f, 4), float3(0, 0, 0), float3(0, 1, 0)));
            points = ApplyTransform(points, Transforms.PerspectiveFovLH(pi_over_4, render.RenderTarget.Height / (float)render.RenderTarget.Width, 0.01f, 10));

            #endregion

            render.DrawPoints(points);
        }

        public static void DrawRoomTest(Raster raster)
        {
            raster.ClearRT(float4(0, 0, 0.2f, 1)); // clear with color dark blue.

            int N = 100000;
            // Create buffer with points to render
            float3[] points = RandomPositionsInBoxSurface(N);

            float4x4 viewMatrix = Transforms.LookAtLH(float3(5f, 4.6f, 2), float3(0, 0, 0), float3(0, 1, 0));
            float4x4 projMatrix = Transforms.PerspectiveFovLH(pi_over_4, raster.RenderTarget.Height / (float)raster.RenderTarget.Width, 0.01f, 10);

            DrawRoom(raster, points, mul(viewMatrix, projMatrix));
        }

        private static void DrawRoom(Raster raster, float3[] boxPoints, float4x4 transform)
        {
            DrawTable(raster, boxPoints, mul(Transforms.Translate(0, 0, 0), transform));
            DrawTable(raster, boxPoints, mul(Transforms.RotateRespectTo(float3(1, 0, 0), float3(0, 1, 0), pi / 2), transform));
        }

        private static void DrawTable(Raster raster, float3[] boxPoints, float4x4 transform)
        {
            DrawTableLeg(raster, boxPoints, mul(Transforms.Translate(0.2f, 0, 0.2f), transform));
            DrawTableLeg(raster, boxPoints, mul(Transforms.Translate(1.6f, 0, 0.2f), transform));
            DrawTableLeg(raster, boxPoints, mul(Transforms.Translate(1.6f, 0, 1.6f), transform));
            DrawTableLeg(raster, boxPoints, mul(Transforms.Translate(0.2f, 0, 1.6f), transform));
            DrawTableTop(raster, boxPoints, mul(Transforms.Translate(0, 2, 0), transform));
        }

        private static void DrawTableTop(Raster raster, float3[] boxPoints, float4x4 transform)
        {
            float4x4 transformingIntoLeg = mul(Transforms.Scale(2.2f, 0.2f, 2.2f), transform);
            DrawBox(raster, boxPoints, transformingIntoLeg);
        }

        private static void DrawTableLeg(Raster raster, float3[] boxPoints, float4x4 transform)
        {
            float4x4 transformingIntoLeg = mul(Transforms.Scale(0.2f, 2, 0.2f), transform);
            DrawBox(raster, boxPoints, transformingIntoLeg);
        }

        private static void DrawBox(Raster raster, float3[] boxPoints, float4x4 transform)
        {
            float4x4 transformingIntoBox = mul(float4x4(
                0.5f, 0, 0, 0,
                0, 0.5f, 0, 0,
                0, 0, 0.5f, 0,
                0.5f, 0.5f, 0.5f, 1
                ), transform);

            float3[] pointsToDraw = ApplyTransform(boxPoints, transformingIntoBox);
            raster.DrawPoints(pointsToDraw);
        }
    }
}
