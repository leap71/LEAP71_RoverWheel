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
		public class WheelTread
		{
			protected Frames		m_aFrames;
			protected float			m_fContourHeight;
			protected float			m_fRefRadius;
			protected ITreadPattern	m_xPattern;


			/// <summary>
			/// Generates the tread of a wheel by revoling the outer radius contour.
			/// The specified tread pattern can either be added as a exposed profile or
			/// subtracted from a solid tread layer.
			/// </summary>
			public WheelTread(	Frames aOuterWheelFrames,
								ITreadPattern xTreadPattern)
			{
				m_aFrames	= aOuterWheelFrames;
				m_xPattern	= xTreadPattern;
            }

			/// <summary>
			/// Generates the tread pattern as an exposed profile.
			/// </summary>
			public Voxels voxGetProfile()
			{
				m_fRefRadius				= 140f;
                m_fContourHeight			= SplineOperations.fGetTotalLength(m_aFrames.aGetPoints(100));
				Voxels voxProfile			= m_xPattern.voxConstruct(	m_fRefRadius,
																		m_fContourHeight,
																		vecTreadTrafo);
				Sh.PreviewVoxels(voxProfile, Cp.clrRandom());
				return voxProfile;
			}

			/// <summary>
			/// Generates the tread by subtracting the pattern from a solid layer.
			/// Thickness (outwards / inwards) of the solid layer can be specified.
			/// </summary>
			public Voxels voxGetTreadLayer(	float fOutwardsThickness	= 3f,
											float fInwardsThickness		= 2f)
			{
                BaseRevolve oTreadBase		= new BaseRevolve(new LocalFrame(), m_aFrames, fInwardsThickness, fOutwardsThickness);
				Voxels voxTreadBase			= oTreadBase.voxConstruct();

				m_fRefRadius				= 140f;
                m_fContourHeight			= SplineOperations.fGetTotalLength(m_aFrames.aGetPoints(100));
				Voxels voxProfile			= m_xPattern.voxConstruct(	m_fRefRadius,
																		m_fContourHeight,
																		vecTreadTrafo);

				voxTreadBase				= Sh.voxSubtract(voxTreadBase, voxProfile);
				Sh.PreviewVoxels(voxTreadBase, Cp.clrRandom());
				return voxTreadBase;
			}

			protected Vector3 vecTreadTrafo(Vector3 vecPt)
			{
				//input is a shape on a cylinder
				float fRadius		= VecOperations.fGetRadius(vecPt);
                float fPhi			= VecOperations.fGetPhi(vecPt);
				float fZ			= vecPt.Z;

				float fLengthRatio	= fZ / m_fContourHeight;
				LocalFrame oFrame	= m_aFrames.oGetLocalFrame(fLengthRatio);
				float fNewZ			= oFrame.vecGetPosition().Z;
				float fNewRadius	= VecOperations.fGetRadius(oFrame.vecGetPosition());

                Vector3 vecTread	= oFrame.vecGetLocalZ();
				Vector3 vecRadial	= oFrame.vecGetLocalX();

				float dRadius		= fRadius - m_fRefRadius;
				Vector3 vecNewPt	= VecOperations.vecRotateAroundZ(new Vector3(fNewRadius, 0, fNewZ) + dRadius * vecRadial, fPhi);
				return vecNewPt;
            }

			public Frames aGetFrames()
			{
				return m_aFrames;
			}
		}
	}
}