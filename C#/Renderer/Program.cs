using System.Linq;
using System.Collections.Generic;
using GMath;
using Rendering;
using System;
using static GMath.Gfx;

namespace Renderer
{
    class Program
    {
        struct MyVertex : IVertex<MyVertex>
        {
            public float3 Position { get; set; }

            public MyVertex Add(MyVertex other)
            {
                return new MyVertex
                {
                    Position = this.Position + other.Position,
                };
            }

            public MyVertex Mul(float s)
            {
                return new MyVertex
                {
                    Position = this.Position * s,
                };
            }
        }

        struct MyProjectedVertex : IProjectedVertex<MyProjectedVertex>
        {
            public float4 Homogeneous { get; set; }

            public MyProjectedVertex Add(MyProjectedVertex other)
            {
                return new MyProjectedVertex
                {
                    Homogeneous = this.Homogeneous + other.Homogeneous
                };
            }

            public MyProjectedVertex Mul(float s)
            {
                return new MyProjectedVertex
                {
                    Homogeneous = this.Homogeneous * s
                };
            }
        }

        static void Main(string[] args)
        {
            Raster<MyVertex, MyProjectedVertex> render = new Raster<MyVertex, MyProjectedVertex>(1024, 512);
            GeneratingMeshes(render);
            //DrawRoomTest(render);
            render.RenderTarget.Save("test.rbm");
            Console.WriteLine("Done.");
        }


        static float3 EvalBezier(float3[] control, float t)
        {
            // DeCasteljau
            if (control.Length == 1)
                return control[0]; // stop condition
            float3[] nestedPoints = new float3[control.Length - 1];
            for (int i = 0; i < nestedPoints.Length; i++)
                nestedPoints[i] = lerp(control[i], control[i + 1], t);
            return EvalBezier(nestedPoints, t);
        }

        static Mesh<MyVertex> createcircle(float z,int d1,int d2)
        {
            return Manifold<MyVertex>.Surface(d1, d2, (u, v) =>
            {
                float alpha = u * 2 * pi;
                float r = sqrt(max(0, 1 - v * v));
                return float3(r * cos(alpha), z, r * sin(alpha));
            });
        }
        static Mesh<MyVertex> createSphere()
        {
            // Parametric representation of a sphere.
            return Manifold<MyVertex>.Surface(30, 30, (u, v) =>
            {
                float alpha = u * 2 * pi;
                float beta = pi / 2 - v * pi;
                return float3(cos(alpha) * cos(beta), sin(beta), sin(alpha) * cos(beta));
            });
        }
        static Mesh<MyVertex> createCilinder(int d1, int d2)
        {
            return Manifold<MyVertex>.Surface(d1, d2, (u, v) =>
              {
                  float alpha = 2 * pi * u;
                  float r = sqrt(max(0, 1 - v * v));
                  float x = cos(alpha);
                  float y = sin(alpha);
                  float z = v;
                  return float3(x, z, y);

              });

        }
        static Mesh<MyVertex> CreateModel()
        {
            // Parametric representation of a sphere.
            //return Manifold<MyVertex>.Surface(30, 30, (u, v) =>
            //{
            //    float alpha = u * 2 * pi;
            //    float beta = pi / 2 - v * pi;
            //    return float3(cos(alpha) * cos(beta), sin(beta), sin(alpha) * cos(beta));
            //});

            // Generative model
            return Manifold<MyVertex>.Generative(30, 30,
                // g function
                u => float3(cos(2 * pi * u), 0, sin(2 * pi * u)),
                // f function
                (p, v) => p + float3(cos(v * pi), 2 * v - 1, 0)
            );

            // Revolution Sample with Bezier
            // float3[] contourn =
            // {
            //     float3(0, -.5f,0),
            //     float3(0.8f, -0.5f,0),
            //     float3(1f, -0.2f,0),
            //     float3(0.6f,1,0),
            //     float3(0,1,0)
            // };
            // return Manifold<MyVertex>.Revolution(30, 30, t => EvalBezier(contourn, t), float3(0, 1, 0));
        }

        private static (Mesh<MyVertex>, Mesh<MyVertex>) createBase()
        {
            Mesh<MyVertex> cil = createCilinder(15, 15);
            Mesh<MyVertex> circ = createcircle(1,15,15);
            cil = cil.ConvertTo(Topology.Lines);
            circ = circ.ConvertTo(Topology.Lines);
            return (cil, circ);
        }
        private static (Mesh<MyVertex>, Mesh<MyVertex>) setbase((Mesh<MyVertex>, Mesh<MyVertex>) _base)
        {
            Mesh<MyVertex> it1 = _base.Item1.Transform(Transforms.Scale((float)0.8, (float)0.4, (float)(0.8)));
            Mesh<MyVertex> it2 = _base.Item2.Transform(Transforms.Scale((float)0.8, (float)0.4, (float)(0.8)));
            it1 = it1.Transform(Transforms.Translate(0, -2, 0));
            it2 = it2.Transform(Transforms.Translate(0, -2, 0));
            return (it1, it2);
        }

        private static (Mesh<MyVertex>, Mesh<MyVertex>) createTubo()
        {
            Mesh<MyVertex> cil1 = createCilinder(3, 3);
            Mesh<MyVertex> cil2 = createCilinder(3, 3);
            cil1 = cil1.ConvertTo(Topology.Lines);
            cil2 = cil2.ConvertTo(Topology.Lines);
            return (cil1, cil2);
        }

        private static (Mesh<MyVertex>, Mesh<MyVertex>) setTubo((Mesh<MyVertex>, Mesh<MyVertex>) _tubo)
        {
            Mesh<MyVertex> t1 = _tubo.Item1.Transform(Transforms.Scale((float)0.04, 2, (float)0.04));
            Mesh<MyVertex> t2 = _tubo.Item2.Transform(Transforms.Scale((float)0.04, 2, (float)0.04));
            t1 = t1.Transform(Transforms.Translate(0, (float)-1.7, 0));
            t2 = t2.Transform(Transforms.Translate(0, (float)0.3, 0));
            t2 = t2.Transform(Transforms.RotateZGrad(-10));
            t2 = t2.Transform(Transforms.RotateXGrad(-10));
            t2 = t2.Transform(Transforms.Translate((float)0.1, 0, 0));

            return (t1, t2);

        }

        private static Mesh<MyVertex> createLampSphere()
        {
            var sphere = createSphere();
            sphere = sphere.ConvertTo(Topology.Lines);
            return sphere;

        }

        private static Mesh<MyVertex> setLampSphere(Mesh<MyVertex> _sphere)
        {
            Mesh<MyVertex> sp = _sphere.Transform(Transforms.Scale((float)0.7, (float)0.7, (float)0.7));
            sp = sp.Transform(Transforms.Translate((float)0.6,(float)1.7,0));
            return sp;
        }
        private static void GeneratingMeshes(Raster<MyVertex, MyProjectedVertex> render)
        {
            render.ClearRT(float4(0, 0, 0.2f, 1)); // clear with color dark blue.

            //base de la lampara
            (Mesh<MyVertex>, Mesh<MyVertex>) _base = createBase();
            _base = setbase(_base);
            /////////////////////////////////////////

            //tubo de la lampara
            (Mesh<MyVertex>, Mesh<MyVertex>) _tubo = createTubo();
            _tubo = setTubo(_tubo);
            ////////////////////////////////////////////////////

            //Bombillo de la lampara
            Mesh<MyVertex> _sphere = createLampSphere();
            _sphere = setLampSphere(_sphere);
            //////////////////////////////////////////////


            /// Convert to a wireframe to render. Right now only lines can be rasterized.
            //_base = _base.ConvertTo(Topology.Lines);

            #region viewing and projecting
            //Transforms.LookAtLH(float3(5f, 2.6f, 4), float3(0, 0, 0), float3(0, 1, 0)
            float4x4 viewMatrix = Transforms.LookAtLH(float3(5f, 2.6f, 4), float3(0, 0, 0), float3(0, 1, 0));
            float4x4 projectionMatrix = Transforms.PerspectiveFovLH(pi_over_4, render.RenderTarget.Height / (float)render.RenderTarget.Width, 0.01f, 10);

            // Define a vertex shader that projects a vertex into the NDC.
            render.VertexShader = v =>
            {
                float4 hPosition = float4(v.Position, 1);
                hPosition = mul(hPosition, viewMatrix);
                hPosition = mul(hPosition, projectionMatrix);
                return new MyProjectedVertex { Homogeneous = hPosition };
            };

            // Define a pixel shader that colors using a constant value
            render.PixelShader = p =>
            {
                return float4(p.Homogeneous.x / 1024.0f, p.Homogeneous.y / 512.0f, 1, 1);
            };

            #endregion

            // Draw the mesh.
            render.DrawMesh(_sphere);
            render.DrawMesh(_base.Item1);
            render.DrawMesh(_base.Item2);
            render.DrawMesh(_tubo.Item1);
            render.DrawMesh(_tubo.Item2);
        }
    }
}
