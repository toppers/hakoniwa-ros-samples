using Hakoniwa.Core.Rpc;
using Hakoniwa.Core.Simulation;
using Hakoniwa.Core.Simulation.Environment;
using Hakoniwa.Core.Utils;
using Hakoniwa.Core.Utils.Logger;
using Hakoniwa.PluggableAsset;
using Hakoniwa.PluggableAsset.Assets;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Hakoniwa.Core
{
    class UnitySimulator : IInsideWorldSimulatior
    {
        public UnitySimulator()
        {

        }
        public void DoSimulation()
        {
            Physics.Simulate(Time.fixedDeltaTime);
        }
    }

    public class WorldController : MonoBehaviour
    {
        private GameObject root;
        public long maxDelayTime = 20000; /* usec */
        private SimulationController simulator = SimulationController.Get();
        private void InitHakoniwa()
        {
            this.root = GameObject.Find("Robot");
#if UNITY_EDITOR
            string filePath = Directory.GetCurrentDirectory();
#else
            string filePath = AppDomain.CurrentDomain.BaseDirectory;
#endif
            Debug.Log(filePath);
            string configPath = filePath + System.IO.Path.DirectorySeparatorChar + "core_config.json";

            AssetConfigLoader.Load(configPath);
            Debug.Log("HakoniwaCore START");
            RpcServer.StartServer(AssetConfigLoader.core_config.core_ipaddr, AssetConfigLoader.core_config.core_portno);
            simulator.RegisterEnvironmentOperation(new UnityEnvironmentOperation());
            simulator.SaveEnvironment();
            simulator.GetLogger().SetFilePath(AssetConfigLoader.core_config.SymTimeMeasureFilePath);

            foreach (Transform child in this.root.transform)
            {
                Debug.Log("child=" + child.name);
                GameObject obj = root.transform.Find(child.name).gameObject;
                IInsideAssetController ctrl = obj.GetComponentInChildren<IInsideAssetController>();
                ctrl.Initialize();
                AssetConfigLoader.AddInsideAsset(ctrl);
                simulator.RegisterInsideAsset(child.name);
            }


            simulator.SetSimulationWorldTime(
                this.maxDelayTime,
                (long)(Time.fixedDeltaTime * 1000000f));
            simulator.SetInsideWorldSimulator(new UnitySimulator());
            Physics.autoSimulation = false;
        }
        void Start()
        {
            try
            {
                this.InitHakoniwa();
            }
            catch (Exception e)
            {
                SimpleLogger.Get().Log(Level.ERROR, e);
                throw e;
            }
        }
        void FixedUpdate()
        {
            try
            {
                this.simulator.Execute();
            }
            catch (Exception e)
            {
                SimpleLogger.Get().Log(Level.ERROR, e);
                throw e;
            }
        }
    }
}
