//
// SPDX-License-Identifier: Apache-2.0
//
// The LEAP 71 ShapeKernel is an open source geometry engine
// specifically for use in Computational Engineering Models (CEM).
//
// For more information, please visit https://leap71.com/shapekernel
// 
// This project is developed and maintained by LEAP 71 - © 2023 by LEAP 71
// https://leap71.com
//
// Computational Engineering will profoundly change our physical world in the
// years ahead. Thank you for being part of the journey.
//
// We have developed this library to be used widely, for both commercial and
// non-commercial projects alike. Therefore, have released it under a permissive
// open-source license.
// 
// The LEAP 71 ShapeKernel is based on the PicoGK compact computational geometry 
// framework. See https://picogk.org for more information.
//
// LEAP 71 licenses this file to you under the Apache License, Version 2.0
// (the "License"); you may not use this file except in compliance with the
// License. You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, THE SOFTWARE IS
// PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
//
// See the License for the specific language governing permissions and
// limitations under the License.   
//


using System.Numerics;
using PicoGK;


namespace Leap71
{
    using ShapeKernel;

    namespace Rover
    {
		/// <summary>
		/// Abstract class from which different rover wheel classes can be derived.
		/// </summary>
		public abstract class RoverWheel
		{

			public static float				m_fHubRadius;
            public static float				m_fOuterRadius;
			public static float				m_fRefWidth;
			protected BaseLens				m_oWheel;
			protected ITreadPattern			m_xTreadPattern;
			protected WheelTread			m_oTread;
			protected static List<Vector3>	m_aFullUpperHeightPoints = new ();
            protected static List<Vector3>	m_aFullLowerHeightPoints = new ();
            protected static Frames			m_aUpperHeightFrames;
            protected static Frames			m_aLowerHeightFrames;
            protected static Frames			m_aInnerRadiusFrames;
            protected static Frames			m_aOuterRadiusFrames;


			/// <summary>
			/// Constructs the final wheel by combining elements, layers and the outer tread.
			/// </summary>
			public abstract Voxels voxConstruct();

			/// <summary>
			/// Helper function to construct the bounding of the full wheel.
			/// This is equal to generating a wheel layer that goes from 0 to 1.
			/// </summary>
            protected Voxels voxGetFullWheel()
			{
				float fStartLengthRatio = 0f;
				float fEndLengthRatio	= 1f;
                float fRefInnerRadius	= m_fHubRadius + fStartLengthRatio * (m_fOuterRadius - m_fHubRadius);
                float fRefOuterRadius	= m_fHubRadius + fEndLengthRatio * (m_fOuterRadius - m_fHubRadius);
                BasePipe oLayer			= new BasePipe(new LocalFrame(), m_fRefWidth, fRefInnerRadius, fRefOuterRadius);
				oLayer.SetLengthSteps(100);
                oLayer.SetRadialSteps(100);
                oLayer.SetPolarSteps(100);
                oLayer.SetTransformation(vecGetWheelLayerTrafo);
                //oLayer.SetTransformation(vecGetDummyTrafo);

                Voxels voxLayer = oLayer.voxConstruct();
                return voxLayer;
            }

			/// <summary>
			/// Returns the specified wheel layer as a voxelfield.
			/// The layer is defined as a wheel layer struct.
			/// </summary>
			public static Voxels voxGetLayer(WheelLayer sLayer)
			{
				return voxGetLayer(sLayer.m_fStartLengthRatio, sLayer.m_fEndLengthRatio);
            }

            /// <summary>
            /// Returns the specified wheel layer as a voxelfield.
            /// The layer is defined via its start and end radius ratio within the wheel.
            /// </summary>
            protected static Voxels voxGetLayer(float fStartLengthRatio, float fEndLengthRatio)
			{
				float fRefInnerRadius	= m_fHubRadius + fStartLengthRatio * (m_fOuterRadius - m_fHubRadius);
                float fRefOuterRadius	= m_fHubRadius + fEndLengthRatio * (m_fOuterRadius - m_fHubRadius);
                BasePipe oLayer			= new BasePipe(new LocalFrame(), m_fRefWidth, fRefInnerRadius, fRefOuterRadius);
				oLayer.SetLengthSteps(100);
                oLayer.SetRadialSteps(100);
                oLayer.SetPolarSteps(100);
                oLayer.SetTransformation(vecGetWheelLayerTrafo);
                //oLayer.SetTransformation(vecGetDummyTrafo);

                Voxels voxLayer = oLayer.voxConstruct();
                Sh.PreviewVoxels(voxLayer, Cp.clrRandom((int)(fStartLengthRatio * 100)), 0.6f);
				return voxLayer;
            }

			/// <summary>
			/// Returns a number of segments that make up a wheel layer.
			/// The number defined the symmetry within the layer.
			/// </summary>
			protected void GenerateLayerSegments(float fStartLengthRatio, float fEndLengthRatio, uint nNumber)
			{
				float fRefInnerRadius	= m_fHubRadius + fStartLengthRatio * (m_fOuterRadius - m_fHubRadius);
                float fRefOuterRadius	= m_fHubRadius + fEndLengthRatio * (m_fOuterRadius - m_fHubRadius);
                float dPhi				= (2f * MathF.PI) / (float)(nNumber);

                for (int i = 0; i < nNumber; i++)
                {
                    float fPhi				= (2f * MathF.PI) / (float)(nNumber) * i;
                    BasePipeSegment oLayer	= new BasePipeSegment(	new LocalFrame(),
																	m_fRefWidth,
																	fRefInnerRadius,
																	fRefOuterRadius,
																	new LineModulation(fPhi),
																	new LineModulation(fPhi + dPhi),
																	BasePipeSegment.EMethod.START_END);
					oLayer.SetLengthSteps(100);
					oLayer.SetRadialSteps(100);
					oLayer.SetPolarSteps(100);
                    oLayer.SetTransformation(vecGetWheelLayerTrafo);
                    //oLayer.SetTransformation(vecGetDummyTrafo);

                    Voxels voxLayer			= oLayer.voxConstruct();
					Sh.PreviewVoxels(voxLayer, Cp.clrRandom((int)nNumber * i), 0.6f);
				}
			}

			/// <summary>
			/// Provides the coordinate transformation that translate from a simple cylindrical space to the curved wheel space.
			/// </summary>
			public static Vector3 vecGetWheelLayerTrafo(Vector3 vecPt)
			{
				float fRadius		= VecOperations.fGetRadius(vecPt);
				float fPhi			= VecOperations.fGetPhi(vecPt);
				float fHeightRatio	= vecPt.Z / m_fRefWidth;
				float fLengthRatio	= (fRadius - m_fHubRadius) / (m_fOuterRadius - m_fHubRadius);
				Vector3 vecNewPt	= vecGetInnerPt(fHeightRatio, fLengthRatio);
				vecNewPt			= VecOperations.vecRotateAroundZ(vecNewPt, fPhi);
				return vecNewPt;
			}

			/// <summary>
			/// Provides a dummy coordinate transformation that (when applied to the wheel) showcases the simple cylindrical design space.
			/// It does not deform the design space, it just shifts it along the z-axis to be compatible with the actual wheel trafo.
			/// </summary>
			public static Vector3 vecGetDummyTrafo(Vector3 vecPt)
			{
				return vecPt - 0.5f * m_fRefWidth * Vector3.UnitZ;
			}

			/// <summary>
			/// Helper function to visualize a 2D slice of the wheel coordinate space.
			/// </summary>
            protected void ShowPlaneGrid()
			{
				uint nSamples			= 100;

				for (float fLengthRatio = 0; fLengthRatio <= 1f; fLengthRatio += 0.1f)
				{
					List<Vector3> aPoints	= new List<Vector3>();
					for (int i = 0; i < nSamples; i++)
					{
						float fHeightRatio = 1f / (float)(nSamples - 1) * i;
						Vector3 vecPt = vecGetInnerPt(fHeightRatio, fLengthRatio);
						aPoints.Add(vecPt);
					}
					Sh.PreviewLine(aPoints, Cp.clrRock);
				}

				for (float fHeightRatio = 0; fHeightRatio <= 1f; fHeightRatio += 0.1f)
				{
					List<Vector3> aPoints	= new List<Vector3>();
					for (int i = 0; i < nSamples; i++)
					{
						float fLengthRatio = 1f / (float)(nSamples - 1) * i;
						Vector3 vecPt = vecGetInnerPt(fHeightRatio, fLengthRatio);
						aPoints.Add(vecPt);
					}
					Sh.PreviewLine(aPoints, Cp.clrRock);
				}
			}

			/// <summary>
			/// Helper function to navigate within wheel coordinate space and
			/// address (conformal) points within the wheel's bounding shape.
			/// </summary>
			static Vector3 vecGetInnerPt(float fHeightRatio, float fLengthRatio)
			{
				Vector3 vecUpper	= m_aUpperHeightFrames.vecGetSpineAlongLength(fLengthRatio);
                Vector3 vecLower	= m_aLowerHeightFrames.vecGetSpineAlongLength(fLengthRatio);

				Vector3 vecOuter	= m_aOuterRadiusFrames.vecGetSpineAlongLength(fHeightRatio);
				Vector3 vecInner	= m_aInnerRadiusFrames.vecGetSpineAlongLength(fHeightRatio);

				Vector3 vecInnerRef = m_aInnerRadiusFrames.vecGetSpineAlongLength(0) + fHeightRatio * (m_aInnerRadiusFrames.vecGetSpineAlongLength(1) - m_aInnerRadiusFrames.vecGetSpineAlongLength(0));
                Vector3 vecOuterRef = m_aOuterRadiusFrames.vecGetSpineAlongLength(0) + fHeightRatio * (m_aOuterRadiusFrames.vecGetSpineAlongLength(1) - m_aOuterRadiusFrames.vecGetSpineAlongLength(0));

				Vector3 vecDInner	= vecInner - vecInnerRef;
                Vector3 vecDOuter	= vecOuter - vecOuterRef;

				Vector3 vecD		= vecDInner + fLengthRatio * (vecDOuter - vecDInner);
				Vector3 vecRef		= vecLower  + fHeightRatio * (vecUpper - vecLower);
				Vector3 vecPt		= vecRef    + vecD;
				return vecPt;
			}

			/// <summary>
			/// Splits the upper and lower modulated surface of the wheel into 2D contours.
			/// The upper contour runs along the top, radially outwards until the outer tread starts.
			/// The outer tread contour covers the most radially outwards portion.
			/// The lower contour runs along the bottom, radially outwards untul the outer tread starts.
			/// The contour point at which the tread starts and ends is defined as where the local contour switches
			/// from being more aligned with the radial direction to being more aligned with the axial direction.
			/// </summary>
			protected void SetOuterPoints()
			{
				// prep upper and lower contour
				List<Vector3> aUpperWheelContour = new List<Vector3>();
				foreach (Vector3 vecPt in m_aFullUpperHeightPoints.GetRange(0, m_aFullUpperHeightPoints.Count - 2))
                {
                    float fNewZ			= vecPt.X;
                    float fNewY			= 0f;
                    float fNewX			= vecPt.Z * (m_fOuterRadius - m_fHubRadius) + m_fHubRadius;
                    Vector3 vecNewPt	= new Vector3(fNewX, fNewY, fNewZ);
                    aUpperWheelContour.Add(vecNewPt);
                }
                aUpperWheelContour = SplineOperations.aGetReparametrizedSpline(aUpperWheelContour, 1f);


                List<Vector3> aLowerWheelContour = new List<Vector3>();
                foreach (Vector3 vecPt in m_aFullLowerHeightPoints.GetRange(0, m_aFullLowerHeightPoints.Count - 2))
                {
                    float fNewZ			= vecPt.X;
                    float fNewY			= 0f;
                    float fNewX			= vecPt.Z * (m_fOuterRadius - m_fHubRadius) + m_fHubRadius;
                    Vector3 vecNewPt	= new Vector3(fNewX, fNewY, fNewZ);
                    aLowerWheelContour.Add(vecNewPt);
                }
                aLowerWheelContour = SplineOperations.aGetReparametrizedSpline(aLowerWheelContour, 1f);


                // filter for orientation
                Vector3 vecTread					= Vector3.UnitZ;
				Vector3 vecRim						= Vector3.UnitX;
				List<Vector3> aOuterRadiusPoints	= new List<Vector3>();
                List<Vector3> aUpperHeightPoints	= new List<Vector3>();
                List<Vector3> aLowerHeightPoints	= new List<Vector3>();
                bool bTread							= false;

                for (int i = 1; i < aUpperWheelContour.Count - 1; i++)
                {
					Vector3 vecPt		= aUpperWheelContour[i];
                    Vector3 vecContour	= (aUpperWheelContour[i - 1] - aUpperWheelContour[i + 1]).Normalize();
					float fRimAlign		= MathF.Abs(Vector3.Dot(vecContour, vecRim));
                    float fTreadAlign	= MathF.Abs(Vector3.Dot(vecContour, vecTread));

                    if (fTreadAlign > fRimAlign)
                    {
                        bTread = true;
                    }
					if (bTread == true)
					{
                        aOuterRadiusPoints.Add(vecPt);
                    }
					else
					{
						aUpperHeightPoints.Add(vecPt);
                    }
                }

				int iCounter			= 0;
				for (int i = aLowerWheelContour.Count - 2; i > 0; i--)
                {
					Vector3 vecPt		= aLowerWheelContour[i];
                    Vector3 vecContour	= (aLowerWheelContour[i + 1] - aLowerWheelContour[i - 1]).Normalize();
					float fRimAlign		= MathF.Abs(Vector3.Dot(vecContour, vecRim));
                    float fTreadAlign	= MathF.Abs(Vector3.Dot(vecContour, vecTread));

					if (iCounter > 20 && fTreadAlign < fRimAlign)
                    {
						bTread = false;
					}
					if (bTread == true)
					{
                        aOuterRadiusPoints.Add(vecPt);
                    }
					else
					{
						aLowerHeightPoints.Insert(0, vecPt);
                    }
					iCounter++;
                }

                aLowerHeightPoints = SplineOperations.aGetReparametrizedSpline(aLowerHeightPoints, 1f);
                aUpperHeightPoints = SplineOperations.aGetReparametrizedSpline(aUpperHeightPoints, 1f);
                aOuterRadiusPoints = SplineOperations.aGetReparametrizedSpline(aOuterRadiusPoints, 1f);

                Sh.PreviewLattice(Sh.latFromPoint(aOuterRadiusPoints[0], 3f), Cp.clrRed);
				Sh.PreviewLattice(Sh.latFromPoint(aOuterRadiusPoints[^1], 3f), Cp.clrRed);
                Sh.PreviewLattice(Sh.latFromLine(aOuterRadiusPoints, 1f), Cp.clrRed);
                Sh.PreviewLattice(Sh.latFromLine(aUpperHeightPoints, 1f), Cp.clrBlack);
                Sh.PreviewLattice(Sh.latFromLine(aLowerHeightPoints, 1f), Cp.clrBlack);

				m_aOuterRadiusFrames = new Frames(aOuterRadiusPoints, Frames.EFrameType.MIN_ROTATION);
                m_aUpperHeightFrames = new Frames(aUpperHeightPoints, Frames.EFrameType.MIN_ROTATION);
                m_aLowerHeightFrames = new Frames(aLowerHeightPoints, Frames.EFrameType.MIN_ROTATION);
            }

			/// <summary>
			/// Creates points of the inner, radial contour which is the wheel hub with a constant radius. 
			/// </summary>
            protected void SetInnerRadiusPoints()
            {
				uint nSamples						= 100;
				List<Vector3> aInnerRadiusPoints	= new List<Vector3>();
				for (int i = 0; i < nSamples; i++)
				{
					float fHeightRatio	= 1f / (float)(nSamples - 1) * i;
                    Vector3 vecPt		= m_oWheel.vecGetSurfacePoint(fHeightRatio, 0f, 0f);
					aInnerRadiusPoints.Add(vecPt);
                }
                Sh.PreviewLattice(Sh.latFromPoint(aInnerRadiusPoints[0], 3f), Cp.clrRuby);
                Sh.PreviewLattice(Sh.latFromPoint(aInnerRadiusPoints[^1], 3f), Cp.clrRuby);
                Sh.PreviewLattice(Sh.latFromLine(aInnerRadiusPoints, 1f), Cp.clrRuby);

                m_aInnerRadiusFrames = new Frames(aInnerRadiusPoints, Frames.EFrameType.MIN_ROTATION);
            }

			/// <summary>
			/// Creates the points for modulating the upper surface of the wheel bounding body.
			/// </summary>
            protected abstract void SetFullUpperHeightPoint();

            /// <summary>
            /// Creates the points for modulating the lower surface of the wheel bounding body.
			/// In this case, the function just mirrors the upper modulation at the z-plane.
            /// </summary>
            protected void SetFullLowerHeightPoints()
			{
                if (m_aFullLowerHeightPoints == null)
				{
					List<Vector3> aUpperHeightPoints = m_aFullUpperHeightPoints;
					List<Vector3> aLowerHeightPoints = new List<Vector3>();

					foreach(Vector3 vecUpper in aUpperHeightPoints)
					{
						Vector3 vecLower = new Vector3(-vecUpper.X, vecUpper.Y, vecUpper.Z);
						aLowerHeightPoints.Add(vecLower);
					}
                    m_aFullLowerHeightPoints = aLowerHeightPoints;
				}
            }

			/// <summary>
			/// Creates a surface modulation from points.
			/// </summary>
			protected SurfaceModulation oGetWidthModulation(List<Vector3> aPoints)
			{
				SurfaceModulation oSurfMod = new (new LineModulation(aPoints, LineModulation.ECoord.X, LineModulation.ECoord.Z));
				return oSurfMod;
            }
		}
	}
}