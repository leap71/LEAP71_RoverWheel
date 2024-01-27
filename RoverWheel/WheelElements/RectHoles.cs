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
		public abstract class WheelElements
		{
            protected float         m_fWallThickness;
            protected WheelLayer    m_sLayer;
            protected uint          m_nSymmetry;

            public WheelElements(   WheelLayer sLayer,
                                    uint nSymmetry,
                                    float fWallThickness)
            {
                m_sLayer            = sLayer;
                m_nSymmetry         = nSymmetry;
                m_fWallThickness    = fWallThickness;
            }

			public abstract Voxels voxConstruct();
        }

		public class RectHoles : WheelElements
        {
			public RectHoles(	WheelLayer  sLayer,
								uint		nSymmetry,
								float		fWallThickness) : base(sLayer, nSymmetry, fWallThickness) { }

            public Voxels voxGetHoles()
			{
				float fStartLengthRatio = m_sLayer.m_fStartLengthRatio;
                float fEndLengthRatio	= m_sLayer.m_fEndLengthRatio;
				float fHubRadius		= RoverWheel.m_fHubRadius;
				float fOuterRadius		= RoverWheel.m_fOuterRadius;
				float fRefWidth			= RoverWheel.m_fRefWidth;

                float fMidLengthRatio	= 0.5f * (fStartLengthRatio + fEndLengthRatio);
                float fRefMidRadius		= fHubRadius + fMidLengthRatio   * (fOuterRadius - fHubRadius);
                float fRefInnerRadius	= fHubRadius + fStartLengthRatio * (fOuterRadius - fHubRadius);
                float fRefOuterRadius	= fHubRadius + fEndLengthRatio   * (fOuterRadius - fHubRadius);

				List<Voxels> aVoxelList = new List<Voxels>();
				for (int i = 0; i < m_nSymmetry; i++)
				{
                    m_fPhiMid           = (2f * MathF.PI) / (float)m_nSymmetry * i;
					LocalFrame oFrame	= new LocalFrame();
                    BaseCylinder oHole	= new BaseCylinder(oFrame, fRefWidth, 1f);
                    oHole.SetLengthSteps(100);
                    oHole.SetRadialSteps(100);
                    oHole.SetPolarSteps(100);
                    oHole.SetTransformation(vecTrafo);

                    aVoxelList.Add(oHole.voxConstruct());
                }

				Voxels voxHoles = Sh.voxUnion(aVoxelList);
                Sh.PreviewVoxels(voxHoles, Cp.clrRandom((int)m_nSymmetry));
				return voxHoles;
            }

            public override Voxels voxConstruct()
            {
                Voxels voxHoles = voxGetHoles();
                Voxels voxLayer = RoverWheel.voxGetLayer(m_sLayer);
                return Sh.voxSubtract(voxLayer, voxHoles);
            }

            protected float m_fPhiMid;
            protected Vector3 vecTrafo(Vector3 vecPt)
			{
				//transform uniform cylinder to rect hole
				float fStartLengthRatio = m_sLayer.m_fStartLengthRatio;
                float fEndLengthRatio	= m_sLayer.m_fEndLengthRatio;
				float fHubRadius		= RoverWheel.m_fHubRadius;
				float fOuterRadius		= RoverWheel.m_fOuterRadius;
                float fRefInnerRadius	= fHubRadius + fStartLengthRatio * (fOuterRadius - fHubRadius);
                float fRefOuterRadius	= fHubRadius + fEndLengthRatio   * (fOuterRadius - fHubRadius);

                vecPt					= vecSupershapeTrafo(vecPt);

                float fRadiusRatio		= (vecPt.Y + 1f) / 2f;                  //-1 to 1
                float fWidthRatio		= vecPt.X;                              //-1 to 1

				float fZ			    = vecPt.Z;
                float fInnerR           = fRefInnerRadius + m_fWallThickness;
                float fOuterR           = fRefOuterRadius - m_fWallThickness;
                float fNewRadius	    = fInnerR + fRadiusRatio * (fOuterR - fInnerR);

				float fCoreGap	        = 0.5f * m_fWallThickness;
                float fMaxPhi           = (MathF.PI) / (float)(m_nSymmetry);
                float fMaxBogen         = fMaxPhi * fNewRadius;
                float fBogen            = fMaxBogen - fCoreGap;
                float dPhi			    = fBogen / fNewRadius;
                float fNewPhi		    = m_fPhiMid + (dPhi * fWidthRatio);
                float fNewZ			    = fZ;

                Vector3 vecNewPt        = VecOperations.vecGetCylPoint(fNewRadius, fNewPhi, fNewZ);

				//transform to wheel space
				return RoverWheel.vecGetWheelLayerTrafo(vecNewPt);
                return RoverWheel.vecGetDummyTrafo(vecNewPt);
            }

            protected Vector3 vecSupershapeTrafo(Vector3 vecPt)
            {
                float fZ            = vecPt.Z;
                float fPhi          = VecOperations.fGetPhi(vecPt);
                float fRadius       = VecOperations.fGetRadius(vecPt);
                float fNewRadius    = fRadius * Uf.fGetSuperShapeRadius(fPhi, 4, 20, 15, 15);
                Vector3 vecNewPt    = VecOperations.vecGetCylPoint(fNewRadius, fPhi, fZ);
                return vecNewPt;
            }
		}
	}
}