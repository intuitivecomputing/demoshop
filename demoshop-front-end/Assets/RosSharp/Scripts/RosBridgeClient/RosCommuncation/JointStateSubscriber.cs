/*
© Siemens AG, 2017-2019
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;
using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{
    public class JointStateSubscriber : Subscriber<Messages.Sensor.JointState>
    {
        public List<string> JointNames;
        public List<JointStateWriter> JointStateWriters;
        [HideInInspector]
        public string[] joint_names;
        [HideInInspector]
        public float[] joint_states;

        protected override void ReceiveMessage(Messages.Sensor.JointState message)
        {
            joint_names = new string[7];
            joint_states = new float[7];
            for (int i = 0; i < message.name.Length; i++)
            {
                joint_names[i] = message.name[i];
                joint_states[i] = (float)message.position[i];
            }
        }
    }
}

