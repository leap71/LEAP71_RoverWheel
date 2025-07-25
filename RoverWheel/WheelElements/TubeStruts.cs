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
		public class TubeStruts : WheelElements
        {
			public TubeStruts(	WheelLayer  sLayer,
								uint		nSymmetry,
								float		fWallThickness) : base(sLayer, nSymmetry, fWallThickness) { }

            public Voxels voxGetStruts()
			{
                // override symmetry
                float fStartLengthRatio         = m_sLayer.m_fStartLengthRatio;
                float fEndLengthRatio	        = m_sLayer.m_fEndLengthRatio;
				float fHubRadius		        = RoverWheel.m_fHubRadius;
				float fOuterRadius		        = RoverWheel.m_fOuterRadius;
                float fRefInnerRadius	        = fHubRadius + fStartLengthRatio * (fOuterRadius - fHubRadius);
                float fRefOuterRadius	        = fHubRadius + fEndLengthRatio   * (fOuterRadius - fHubRadius);
                float fMidRadius                = 0.5f * (fRefInnerRadius + fRefOuterRadius);
                float fRange                    = fRefOuterRadius - fRefInnerRadius;
                m_nSymmetry                     = Math.Max(m_nSymmetry, (uint)(2f * MathF.PI * fMidRadius / fRange));


                float fRefWidth			        = RoverWheel.m_fRefWidth;
				uint nCircleSamples		        = 100;
				List<Voxels> aVoxelList         = new List<Voxels>();
                List<float> aZList              = new List<float>() { fRefWidth - 0.5f * m_fWallThickness, 0.5f * m_fWallThickness };

                foreach (float fZ in aZList)
                {
                    for (int i = 0; i < m_nSymmetry; i++)
				    {
					    m_fPhiMid               = (2f * MathF.PI) / (float)m_nSymmetry * i;
					    Lattice oLattice	    = new Lattice();
					    for(int n = 0; n < nCircleSamples; n++)
					    {
						    float fLocalPhi1    = (2f * MathF.PI) / (float)(nCircleSamples) * (n - 1);
                            Vector3 vecPt1      = VecOperations.vecGetCylPoint(1, fLocalPhi1, fZ);
                            vecPt1              = vecTrafo(vecPt1);

                            float fLocalPhi2    = (2f * MathF.PI) / (float)(nCircleSamples) * (n - 0);
                            Vector3 vecPt2      = VecOperations.vecGetCylPoint(1, fLocalPhi2, fZ);
                            vecPt2              = vecTrafo(vecPt2);
                            oLattice.AddBeam(vecPt1, 0.5f * m_fWallThickness, vecPt2, 0.5f * m_fWallThickness);
                        }
					    aVoxelList.Add(new Voxels(oLattice));
                    }
				}

                Voxels oVoxels		= Voxels.voxCombineAll(aVoxelList);
                oVoxels.ProjectZSlice(aZList[0], aZList[^1]);

                Mesh oMesh			= new Mesh(oVoxels);
				Voxels voxStruts	= new Voxels(MeshUtility.mshApplyTransformation(oMesh, RoverWheel.vecGetWheelLayerTrafo));
                Sh.PreviewVoxels(voxStruts, Cp.clrRandom((int)m_nSymmetry));
				return voxStruts;
            }

            public override Voxels voxConstruct()
            {
                Voxels voxStruts = voxGetStruts();
                return voxStruts;
            }

            float m_fPhiMid;
            Vector3 vecTrafo(Vector3 vecPt)
			{
				// transform uniform cylinder to rect hole
				float fStartLengthRatio = m_sLayer.m_fStartLengthRatio;
                float fEndLengthRatio	= m_sLayer.m_fEndLengthRatio;
				float fHubRadius		= RoverWheel.m_fHubRadius;
				float fOuterRadius		= RoverWheel.m_fOuterRadius;
                float fRefInnerRadius	= fHubRadius + fStartLengthRatio * (fOuterRadius - fHubRadius);
                float fRefOuterRadius	= fHubRadius + fEndLengthRatio   * (fOuterRadius - fHubRadius);

                float fRadiusRatio		= (vecPt.Y + 1f) / 2f;                  //-1 to 1
                float fWidthRatio		= vecPt.X;                              //-1 to 1

                float fInnerR           = fRefInnerRadius;
                float fOuterR           = fRefOuterRadius;
                float fNewRadius	    = fInnerR + fRadiusRatio * (fOuterR - fInnerR);

				float fCoreGap	        = 0.5f * m_fWallThickness;
                float fMaxPhi           = (MathF.PI) / (float)(m_nSymmetry);
                float fMaxBogen         = fMaxPhi * fNewRadius;
                float fBogen            = fMaxBogen - fCoreGap;
                float dPhi			    = fBogen / fNewRadius;
                float fNewPhi		    = m_fPhiMid + (dPhi * fWidthRatio);

                Vector3 vecNewPt        = VecOperations.vecGetCylPoint(fNewRadius, fNewPhi, vecPt.Z);
                return vecNewPt;
            }
		}
	}
}