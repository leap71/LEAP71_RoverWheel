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
		using static BaseShape;

		public class TreadPattern_02 : ITreadPattern
        {
            public Voxels voxConstruct(float fRefRadius,
                                        float fContourHeight,
                                        TrafoFunc oTreadTrafoFunc)
            {
				uint nRibs					= 50;
				Lattice oLattice			= new Lattice();
				for (int i = 0; i < nRibs; i++)
				{
					float fPhi				= (2f * MathF.PI) / (float)(nRibs) * i;
					List<Vector3> aPoints	= new List<Vector3>();
					float dZ				= fContourHeight / 7f;
					float dPhi				= 0.2f;
					int iCounter			= 0;

                    for (float fZ = 0; fZ <= fContourHeight; fZ += dZ)
					{
						if (iCounter % 2 == 1)
						{
							aPoints.Add(VecOperations.vecGetCylPoint(fRefRadius, fPhi + dPhi, fZ));
                        }
						else
						{
							aPoints.Add(VecOperations.vecGetCylPoint(fRefRadius, fPhi, fZ));
                        }
						iCounter++;
                    }
					aPoints = SplineOperations.aGetReparametrizedSpline(aPoints, 1f);

					for (float dRadiusRatio = 0; dRadiusRatio < 1f; dRadiusRatio += 0.01f)
					{
						foreach (Vector3 vecPt in aPoints)
						{
							float fLengthRatio	= Uf.fLimitValue(vecPt.Z / fContourHeight, 0f, 1f);
							float fMaxRadius	= Uf.fTransSmooth(2f, 8f, vecPt.Z, 15f, 3f);
                            fMaxRadius			= Uf.fTransSmooth(fMaxRadius, 2f, vecPt.Z, fContourHeight - 15f, 3f);
                            float dRadius		= dRadiusRatio * fMaxRadius;
                            Vector3 vecNewPt	= VecOperations.vecUpdateRadius(vecPt, dRadius);
							vecNewPt			= oTreadTrafoFunc(vecNewPt);
							oLattice.AddSphere(vecNewPt, 2f);
						}
					}
				}
                Voxels voxProfile		= new Voxels(oLattice);
				return voxProfile;
            }
		}
	}
}