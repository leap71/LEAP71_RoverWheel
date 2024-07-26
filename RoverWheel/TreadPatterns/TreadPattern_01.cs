﻿//
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


using PicoGK;


namespace Leap71
{
	using ShapeKernel;

	namespace Rover
	{
		using static BaseShape;

		public interface ITreadPattern
		{
			public Voxels voxConstruct(float fRefRadius,
									   float fContourHeight,
                                       fnVertexTransformation oTreadTrafoFunc);
		}

		public class TreadPattern_01 : ITreadPattern
        {
			protected float m_fRefRadius;

            public Voxels voxConstruct(	float fRefRadius,
                                        float fContourHeight,
                                        fnVertexTransformation oTreadTrafoFunc)
			{
                m_fRefRadius			= fRefRadius;
                BasePipe oProfile		= new BasePipe(new LocalFrame(), fContourHeight);
				oProfile.SetRadius(new SurfaceModulation(fRefRadius), new SurfaceModulation(fGetProfileHeight));
				Voxels voxRawProfile	= oProfile.voxConstruct();

				oProfile.SetTransformation(oTreadTrafoFunc);
                Voxels voxProfile		= oProfile.voxConstruct();
				return voxProfile;
            }

			protected float fGetProfileHeight(float fPhi, float fLengthRatio)
			{
                float fProfile1 = Uf.fLimitValue(10f * MathF.Cos(50f * fLengthRatio), 0f, 8f);
                float fProfile2 = Uf.fLimitValue(10f * MathF.Cos(50f * fPhi), 0f, 8f);
                return m_fRefRadius + fProfile1 + fProfile2;
            }
		}
	}
}