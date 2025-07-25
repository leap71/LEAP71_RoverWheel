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
		public class SpiralStruts : WheelElements
        {
			public SpiralStruts(	WheelLayer  oLayer,
									uint		nSymmetry,
									float		fWallThickness) : base(oLayer, nSymmetry, fWallThickness) { }

            public override Voxels voxConstruct()
			{
				// override symmetry
                float fStartLengthRatio         = m_sLayer.m_fStartLengthRatio;
                float fEndLengthRatio	        = m_sLayer.m_fEndLengthRatio;
				float fHubRadius		        = RoverWheel.m_fHubRadius;
				float fOuterRadius		        = RoverWheel.m_fOuterRadius;
                float fRefInnerRadius	        = fHubRadius + fStartLengthRatio * (fOuterRadius - fHubRadius);
                float fRefOuterRadius	        = fHubRadius + fEndLengthRatio   * (fOuterRadius - fHubRadius);
                float fRefMidRadius				= 0.5f * (fRefInnerRadius + fRefOuterRadius);
                float fRange                    = fRefOuterRadius - fRefInnerRadius;
				float fRefWidth					= RoverWheel.m_fRefWidth;
                float fMidLengthRatio			= 0.5f * (fStartLengthRatio + fEndLengthRatio);
                m_nSymmetry						= Math.Max(m_nSymmetry, (uint)(2f * MathF.PI * fRefMidRadius / fRange));

				List<Voxels> aVoxelList			= new List<Voxels>();
				float dPhi						= 2f * (2f * MathF.PI) / (float)m_nSymmetry;
				float dR						= fRefOuterRadius - fRefInnerRadius;
                List<float> aZList				= new List<float>() { fRefWidth - 0.5f * m_fWallThickness, 0.5f * m_fWallThickness };

                foreach (float fZ in aZList)
                {
                    for (int i = 0; i < m_nSymmetry; i++)
					{
						float fPhi			= (2f * MathF.PI) / (float)m_nSymmetry * i;

						Vector3 vecPt1		= VecOperations.vecGetCylPoint(fRefInnerRadius, fPhi - 0.5f * dPhi, fZ);
						Vector3 vecRadial1	= VecOperations.vecGetPlanarDir(vecPt1);

						Vector3 vecPt2		= VecOperations.vecGetCylPoint(fRefInnerRadius, fPhi, fZ);
						Vector3 vecRadial2	= VecOperations.vecGetPlanarDir(vecPt2);

						Vector3 vecPt3		= VecOperations.vecGetCylPoint(fRefInnerRadius, fPhi + 0.5f * dPhi, fZ);
						Vector3 vecRadial3	= VecOperations.vecGetPlanarDir(vecPt3);

						List<Vector3> aControlPoints = new List<Vector3>();
						aControlPoints.Add(vecPt1 + 0.0f * dR * vecRadial1);
						aControlPoints.Add(vecPt1 + 0.3f * dR * vecRadial1);
						aControlPoints.Add(vecPt2 + 0.4f * dR * vecRadial2);
						aControlPoints.Add(vecPt3 + 0.7f * dR * vecRadial3);
						aControlPoints.Add(vecPt3 + 1.0f * dR * vecRadial3);

						List<Vector3> aPoints = SplineOperations.aGetNURBSpline(aControlPoints, 500);
						aVoxelList.Add(new Voxels(Sh.latFromLine(aPoints, 0.5f * m_fWallThickness)));
					}
				}

				Voxels oVoxels		= Voxels.voxCombineAll(aVoxelList);
                oVoxels.ProjectZSlice(aZList[0], aZList[^1]);

                Mesh oMesh			= new Mesh(oVoxels);
				Voxels voxStruts	= new Voxels(MeshUtility.mshApplyTransformation(oMesh, RoverWheel.vecGetWheelLayerTrafo));
                Sh.PreviewVoxels(voxStruts, Cp.clrRandom((int)m_nSymmetry));
				return voxStruts;
            }
		}
	}
}