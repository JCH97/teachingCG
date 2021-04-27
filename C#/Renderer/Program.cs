using System.Linq;
using System.Collections.Generic;
using GMath;
using Rendering;
using System;
using System.Diagnostics;
using static GMath.Gfx;

namespace Renderer
{
    class Program
    {

        static void CreateScene(Scene<float3> scene)
        {
            // Adding elements of the scene
            scene.Add(Raycasting.UnitarySphere, Transforms.Translate(0, 1, 0));
            scene.Add(Raycasting.PlaneXZ, Transforms.Identity);
        }

        struct PositionNormal : INormalVertex<PositionNormal>
        {
            public float3 Position { get; set; }
            public float3 Normal { get; set; }

            public PositionNormal Add(PositionNormal other)
            {
                return new PositionNormal
                {
                    Position = this.Position + other.Position,
                    Normal = this.Normal + other.Normal
                };
            }

            public PositionNormal Mul(float s)
            {
                return new PositionNormal
                {
                    Position = this.Position * s,
                    Normal = this.Normal * s
                };
            }

            public PositionNormal Transform(float4x4 matrix)
            {
                float4 p = float4(Position, 1);
                p = mul(p, matrix);

                float4 n = float4(Normal, 0);
                n = mul(n, matrix);

                return new PositionNormal
                {
                    Position = p.xyz / p.w,
                    Normal = n.xyz
                };
            }
        }

        static void CreateScene(Scene<PositionNormal> scene)
        {
            // Adding elements of the scene
            scene.Add(Raycasting.UnitarySphere.AttributesMap(a => new PositionNormal { Position = a, Normal = normalize(a) }),
                Transforms.Translate(0, 1, 0));
            scene.Add(Raycasting.PlaneXZ.AttributesMap(a => new PositionNormal { Position = a, Normal = float3(0, 1, 0) }),
                Transforms.Identity);
        }

        /// <summary>
        /// Payload used to pick a color from a hit intersection
        /// </summary>
        struct MyRayPayload
        {
            public float3 Color;
        }

        /// <summary>
        /// Payload used to flag when a ray was shadowed.
        /// </summary>
        struct ShadowRayPayload
        {
            public bool Shadowed;
        }

        static void SimpleRaycast(Texture2D texture)
        {
            Raytracer<MyRayPayload, float3> raycaster = new Raytracer<MyRayPayload, float3>();

            // View and projection matrices
            float4x4 viewMatrix = Transforms.LookAtLH(float3(2, 1f, 4), float3(0, 0, 0), float3(0, 1, 0));
            float4x4 projectionMatrix = Transforms.PerspectiveFovLH(pi_over_4, texture.Height / (float)texture.Width, 0.01f, 20);

            Scene<float3> scene = new Scene<float3>();
            CreateScene(scene);

            raycaster.OnClosestHit += delegate (IRaycastContext context, float3 attribute, ref MyRayPayload payload)
            {
                payload.Color = attribute;
            };

            for (float px = 0.5f; px < texture.Width; px++)
                for (float py = 0.5f; py < texture.Height; py++)
                {
                    RayDescription ray = RayDescription.FromScreen(px, py, texture.Width, texture.Height, inverse(viewMatrix), inverse(projectionMatrix), 0, 1000);

                    MyRayPayload coloring = new MyRayPayload();

                    raycaster.Trace(scene, ray, ref coloring);

                    texture.Write((int)px, (int)py, float4(coloring.Color, 1));
                }
        }

        static void LitRaycast(Texture2D texture)
        {
            // Scene Setup
            float3 CameraPosition = float3(3, 2f, 4);
            float3 LightPosition = float3(3, 5, -2);
            // View and projection matrices
            float4x4 viewMatrix = Transforms.LookAtLH(CameraPosition, float3(0, 1, 0), float3(0, 1, 0));
            float4x4 projectionMatrix = Transforms.PerspectiveFovLH(pi_over_4, texture.Height / (float)texture.Width, 0.01f, 20);

            Scene<PositionNormal> scene = new Scene<PositionNormal>();
            CreateScene(scene);

            // Raycaster to trace rays and check for shadow rays.
            Raytracer<ShadowRayPayload, PositionNormal> shadower = new Raytracer<ShadowRayPayload, PositionNormal>();
            shadower.OnAnyHit += delegate (IRaycastContext context, PositionNormal attribute, ref ShadowRayPayload payload)
            {
                // If any object is found in ray-path to the light, the ray is shadowed.
                payload.Shadowed = true;
                // No neccessary to continue checking other objects
                return HitResult.Stop;
            };

            // Raycaster to trace rays and lit closest surfaces
            Raytracer<MyRayPayload, PositionNormal> raycaster = new Raytracer<MyRayPayload, PositionNormal>();
            raycaster.OnClosestHit += delegate (IRaycastContext context, PositionNormal attribute, ref MyRayPayload payload)
            {
                // Move geometry attribute to world space
                attribute = attribute.Transform(context.FromGeometryToWorld);

                float3 V = normalize(CameraPosition - attribute.Position);
                float3 L = normalize(LightPosition - attribute.Position);
                float lambertFactor = max(0, dot(attribute.Normal, L));

                // Check ray to light...
                ShadowRayPayload shadow = new ShadowRayPayload();
                shadower.Trace(scene,
                    RayDescription.FromTo(attribute.Position + attribute.Normal * 0.001f, // Move an epsilon away from the surface to avoid self-shadowing 
                    LightPosition), ref shadow);

                payload.Color = shadow.Shadowed ? float3(0, 0, 0) : float3(1, 1, 1) * lambertFactor;
            };
            raycaster.OnMiss += delegate (IRaycastContext context, ref MyRayPayload payload)
            {
                payload.Color = float3(0, 0, 1); // Blue, as the sky.
            };

            /// Render all points of the screen
            for (int px = 0; px < texture.Width; px++)
                for (int py = 0; py < texture.Height; py++)
                {
                    RayDescription ray = RayDescription.FromScreen(px + 0.5f, py + 0.5f, texture.Width, texture.Height, inverse(viewMatrix), inverse(projectionMatrix), 0, 1000);

                    MyRayPayload coloring = new MyRayPayload();

                    raycaster.Trace(scene, ray, ref coloring);

                    texture.Write(px, py, float4(coloring.Color, 1));
                }
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

        static Mesh<MyVertex> createcircle(float z, int d1, int d2)
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
        static Mesh<PositionNormal> CreateModelLudwing()
        {
            // Revolution Sample with Bezier
            float3[] contourn =
            {
                float3(0, -.5f,0),
                float3(0.8f, -0.5f,0),
                float3(1f, -0.2f,0),
                float3(0.6f,1,0),
                float3(0,1,0)
            };

            /// Creates the model using a revolution of a bezier.
            /// Only Positions are updated.
            var model = Manifold<PositionNormal>.Revolution(30, 20, t => EvalBezier(contourn, t), float3(0, 1, 0)).Weld();
            model.ComputeNormals();
            return model;
        }

        static Mesh<PositionNormal> CreateSea()
        {
            /// Creates the model using a revolution of a bezier.
            /// Only Positions are updated.
            var model = Manifold<PositionNormal>.Surface(30, 20, (u, v) => 2 * float3(2 * u - 1, sin(u * 15) * 0.02f + cos(v * 13 + u * 16) * 0.03f, 2 * v - 1)).Weld();
            model.ComputeNormals();
            return model;
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
            Mesh<MyVertex> circ = createcircle(1, 15, 15);
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
            sp = sp.Transform(Transforms.Translate((float)0.6, (float)1.7, 0));
            return sp;
        }
        private static void GeneratingMeshes(Raster<MyVertex, MyProjectedVertex> render)
        {
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
        }

        public delegate float3 BRDF(float3 N, float3 Lin, float3 Lout);

        static BRDF LambertBRDF(float3 diffuse)
        {
            return (N, Lin, Lout) => diffuse / pi;
        }

        static BRDF BlinnBRDF(float3 specular, float power)
        {
            return (N, Lin, Lout) =>
            {
                float3 H = normalize(Lin + Lout);
                return specular * pow(max(0, dot(H, N)), power) * (power + 2) / two_pi;
            };
        }

        static BRDF Mixture(BRDF f1, BRDF f2, float alpha)
        {
            return (N, Lin, Lout) => lerp(f1(N, Lin, Lout), f2(N, Lin, Lout), alpha);
        }

        static void CreateMeshScene(Scene<PositionNormal> scene)
        {
            var sea = CreateSea();
            scene.Add(sea.AsRaycast(RaycastingMeshMode.Grid), Transforms.Identity);

            var egg = CreateModel();
            scene.Add(egg.AsRaycast(RaycastingMeshMode.Grid), Transforms.Translate(0, 0.5f, 0));
        }


        static void RaycastingMesh(Texture2D texture)
        {
            // Scene Setup
            float3 CameraPosition = float3(3, 2f, 4);
            float3 LightPosition = float3(3, 5, -2);
            float3 LightIntensity = float3(1, 1, 1) * 100;

            // View and projection matrices
            float4x4 viewMatrix = Transforms.LookAtLH(CameraPosition, float3(0, 1, 0), float3(0, 1, 0));
            float4x4 projectionMatrix = Transforms.PerspectiveFovLH(pi_over_4, texture.Height / (float)texture.Width, 0.01f, 20);

            Scene<PositionNormal> scene = new Scene<PositionNormal>();
            CreateMeshScene(scene);

            BRDF[] brdfs =
            {
                LambertBRDF(float3(0.4f,0.5f,1f)),  // see
                Mixture(LambertBRDF(float3(0.7f,0.7f,0.3f)), BlinnBRDF(float3(1,1,1), 70), 0.3f), // egg
            };

            // Raycaster to trace rays and check for shadow rays.
            Raytracer<ShadowRayPayload, PositionNormal> shadower = new Raytracer<ShadowRayPayload, PositionNormal>();
            shadower.OnAnyHit += delegate (IRaycastContext context, PositionNormal attribute, ref ShadowRayPayload payload)
            {
                // If any object is found in ray-path to the light, the ray is shadowed.
                payload.Shadowed = true;
                // No neccessary to continue checking other objects
                return HitResult.Stop;
            };

            // Raycaster to trace rays and lit closest surfaces
            Raytracer<MyRayPayload, PositionNormal> raycaster = new Raytracer<MyRayPayload, PositionNormal>();
            raycaster.OnClosestHit += delegate (IRaycastContext context, PositionNormal attribute, ref MyRayPayload payload)
            {
                // Move geometry attribute to world space
                attribute = attribute.Transform(context.FromGeometryToWorld);

                float3 V = normalize(CameraPosition - attribute.Position);
                float3 L = (LightPosition - attribute.Position);
                float d = length(L);
                L /= d; // normalize direction to light reusing distance to light

                float3 N = attribute.Normal;

                float lambertFactor = max(0, dot(N, L));

                // Check ray to light...
                ShadowRayPayload shadow = new ShadowRayPayload();
                shadower.Trace(scene,
                    RayDescription.FromDir(attribute.Position + N * 0.001f, // Move an epsilon away from the surface to avoid self-shadowing 
                    L), ref shadow);

                float3 Intensity = (shadow.Shadowed ? 0.0f : 1.0f) * LightIntensity / (d * d);

                payload.Color = brdfs[context.GeometryIndex](N, L, V) * Intensity * lambertFactor;
            };
            raycaster.OnMiss += delegate (IRaycastContext context, ref MyRayPayload payload)
            {
                payload.Color = float3(0, 0, 1); // Blue, as the sky.
            };

            /// Render all points of the screen
            for (int px = 0; px < texture.Width; px++)
                for (int py = 0; py < texture.Height; py++)
                {
                    int progress = (px * texture.Height + py);
                    if (progress % 1000 == 0)
                    {
                        Console.Write("\r" + progress * 100 / (float)(texture.Width * texture.Height) + "%            ");
                    }

                    RayDescription ray = RayDescription.FromScreen(px + 0.5f, py + 0.5f, texture.Width, texture.Height, inverse(viewMatrix), inverse(projectionMatrix), 0, 1000);

                    MyRayPayload coloring = new MyRayPayload();

                    raycaster.Trace(scene, ray, ref coloring);

                    texture.Write(px, py, float4(coloring.Color, 1));
                }
        }

        static void Main(string[] args)
        {
            Stopwatch stopwatch = new Stopwatch();

            stopwatch.Start();

            // Texture to output the image.
            Texture2D texture = new Texture2D(512, 512);

            //SimpleRaycast(texture);
            //LitRaycast(texture);
            RaycastingMesh(texture);

            stopwatch.Stop();

            texture.Save("test.rbm");

            Console.WriteLine("Done. Rendered in " + stopwatch.ElapsedMilliseconds + " ms");
            // Draw the mesh.
            render.DrawMesh(_sphere);
            render.DrawMesh(_base.Item1);
            render.DrawMesh(_base.Item2);
            render.DrawMesh(_tubo.Item1);
            render.DrawMesh(_tubo.Item2);
        }
    }
}