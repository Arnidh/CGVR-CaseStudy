// CS4104 - Computer Graphics & Virtual Reality | Case Study 4: Behavioural Simulation & Crowd Dynamics
// EvacuationDemo.cs  -  builds the whole metro-station scene from code, renders the crowd and draws the UI.
//
// HOW TO USE: put both .cs files anywhere inside Assets/, open ANY scene (even an empty one) and press Play.
// A [RuntimeInitializeOnLoadMethod] creates everything automatically - no prefabs, textures or setup needed.
// Works in the Built-in, URP and HDRP pipelines. Uses only IMGUI + C#, so it works with mouse OR touch
// and with either Unity input-system setting.
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using UnityEngine.Rendering;
using MetroEvac;
using Debug = UnityEngine.Debug;

public class EvacuationDemo : MonoBehaviour
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Boot()
    {
        if (GameObject.Find("MetroEvacuationDemo") != null) return;
        new GameObject("MetroEvacuationDemo").AddComponent<EvacuationDemo>();
    }

    // ---------------- simulation ----------------
    CrowdSim sim;
    float acc;                               // fixed-timestep accumulator
    int speedMul = 1;
    int agentTarget = 300;
    bool paused;
    readonly Stopwatch watch = new Stopwatch();
    float simMs, frameMs = 16f, fps = 60f;

    // ---------------- scene objects ----------------
    Material baseMat;
    readonly Material[] stateMats = new Material[4];
    Transform crowd;
    Mesh capsuleMesh, cubeMesh;
    int capsuleTris, cubeTris;
    readonly List<Transform> agentTr = new List<Transform>();
    readonly List<Renderer> agentRend = new List<Renderer>();
    readonly List<MeshFilter> agentMF = new List<MeshFilter>();
    readonly List<int> agentMat = new List<int>();
    readonly List<bool> agentLow = new List<bool>();
    readonly List<float> walkPhase = new List<float>();
    readonly GameObject[] barrierGO = new GameObject[CrowdSim.ExitCount];

    // ---------------- rendering optimisation (LOD + frustum statistics) ----------------
    bool useLOD;
    float lodDist = 30f;
    int inView, agentTris;
    readonly Plane[] planes = new Plane[6];

    // ---------------- camera ----------------
    Camera cam, camR;
    int viewMode;                            // 0 = top-down, 1 = angled, 2 = first-person (agent eyes)
    float zoom = 50f, yaw, lookYaw, lookPitch;
    bool stereo;
    const float Ipd = 0.064f;                // inter-pupillary distance (m)
    int viewIdx;

    // ---------------- audio ----------------
    readonly List<AudioSource> sirens = new List<AudioSource>();
    AudioSource murmur;
    bool soundOn = true;

    // ---------------- UI / scenario / logging ----------------
    bool showUI = true;
    Vector2 scroll;
    GUIStyle labelStyle;
    string benchText = "", scenarioLog = "", exportMsg = "";
    bool benchRequested, scenarioOn;
    readonly bool[] scenFired = new bool[4];
    readonly List<string> log = new List<string>();
    float nextLog; bool finalLogged;

    // =====================================================================
    void Start()
    {
        Application.runInBackground = true;
#if UNITY_ANDROID || UNITY_IOS
        Application.targetFrameRate = 60;
        Screen.sleepTimeout = SleepTimeout.NeverSleep;
#endif
        sim = new CrowdSim(1);
        SetupCameras();
        SetupLighting();
        baseMat = GetBaseMaterial();
        stateMats[0] = Mat(new Color(0.45f, 0.70f, 1.00f));   // waiting    - blue
        stateMats[1] = Mat(new Color(1.00f, 0.85f, 0.20f));   // alerted    - yellow
        stateMats[2] = Mat(new Color(0.25f, 0.85f, 0.35f));   // evacuating - green
        stateMats[3] = Mat(new Color(0.95f, 0.15f, 0.15f));   // panicked   - red
        BuildStation();
        crowd = new GameObject("Crowd").transform;
        SetupAudio();
        ResetCrowd();
    }

    // ---------------- helpers ----------------
    Material GetBaseMaterial()
    {
        var rp = GraphicsSettings.currentRenderPipeline;          // URP / HDRP -> the pipeline's own default material
        if (rp != null && rp.defaultMaterial != null) return rp.defaultMaterial;
        var t = GameObject.CreatePrimitive(PrimitiveType.Cube);   // Built-in pipeline
        var m = t.GetComponent<Renderer>().sharedMaterial;
        Destroy(t);
        return m;
    }

    Material Mat(Color c) { var m = new Material(baseMat); m.color = c; return m; }

    GameObject Prim(PrimitiveType type, string name, Vector3 pos, Vector3 scale, Material m, Transform parent)
    {
        var g = GameObject.CreatePrimitive(type);
        g.name = name; g.transform.SetParent(parent, false);
        g.transform.localPosition = pos; g.transform.localScale = scale;
        var r = g.GetComponent<Renderer>(); r.sharedMaterial = m;
        r.shadowCastingMode = ShadowCastingMode.Off; r.receiveShadows = false;
        var col = g.GetComponent<Collider>(); if (col != null) Destroy(col);   // no physics needed -> cheaper
        return g;
    }

    static string F(float v, string f) { return v.ToString(f, System.Globalization.CultureInfo.InvariantCulture); }

    void SetupLighting()
    {
        RenderSettings.ambientMode = AmbientMode.Flat;
        RenderSettings.ambientLight = new Color(0.62f, 0.62f, 0.66f);
        if (RenderSettings.sun == null)
        {
            var l = new GameObject("Directional Light").AddComponent<Light>();
            l.type = LightType.Directional; l.intensity = 0.9f; l.shadows = LightShadows.None;
            l.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
        }
    }

    void SetupCameras()
    {
        cam = Camera.main;
        if (cam == null)
        {
            var g = new GameObject("Main Camera"); g.tag = "MainCamera";
            cam = g.AddComponent<Camera>(); g.AddComponent<AudioListener>();
        }
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.05f, 0.06f, 0.09f);
        cam.nearClipPlane = 0.1f; cam.farClipPlane = 300f; cam.fieldOfView = 50f;
        if (cam.GetComponent<AudioListener>() == null) cam.gameObject.AddComponent<AudioListener>();
        var g2 = new GameObject("RightEyeCamera");
        camR = g2.AddComponent<Camera>();
        camR.CopyFrom(cam); camR.enabled = false;
    }

    // ---------------- station geometry (visuals mirror the simulation obstacles) ----------------
    void BuildStation()
    {
        var root = new GameObject("Station").transform;
        var floorM = Mat(new Color(0.80f, 0.80f, 0.82f));
        var wallM = Mat(new Color(0.93f, 0.90f, 0.82f));
        var pillarM = Mat(new Color(0.65f, 0.65f, 0.70f));
        var kioskM = Mat(new Color(0.15f, 0.55f, 0.60f));
        var pitM = Mat(new Color(0.12f, 0.12f, 0.14f));
        var trainM = Mat(new Color(0.30f, 0.45f, 0.75f));
        var stripeM = Mat(new Color(1.00f, 0.85f, 0.10f));
        var exitM = Mat(new Color(0.10f, 0.85f, 0.30f));
        var streetM = Mat(new Color(0.22f, 0.26f, 0.22f));
        var barrierM = Mat(new Color(0.90f, 0.10f, 0.08f));

        Prim(PrimitiveType.Cube, "Street", new Vector3(0, -0.06f, 0), new Vector3(100, 0.1f, 80), streetM, root);
        var floor = Prim(PrimitiveType.Cube, "Floor", new Vector3(0, -0.05f, 0), new Vector3(62, 0.1f, 42), floorM, root);
        cubeMesh = floor.GetComponent<MeshFilter>().sharedMesh;          // cached for the low-detail agent mesh
        cubeTris = (int)(cubeMesh.GetIndexCount(0) / 3);

        foreach (var o in sim.StaticObstacles)
        {
            switch (o.Kind)
            {
                case "wall":
                    Prim(PrimitiveType.Cube, "Wall", new Vector3(o.X, 2f, o.Z), new Vector3(o.HX * 2f, 4f, o.HZ * 2f), wallM, root); break;
                case "pit":
                    Prim(PrimitiveType.Cube, "TrackBed", new Vector3(o.X, 0.04f, o.Z), new Vector3(o.HX * 2f, 0.1f, o.HZ * 2f), pitM, root);
                    Prim(PrimitiveType.Cube, "Train", new Vector3(o.X, 1.7f, o.Z), new Vector3(o.HX * 2f - 2f, 3.2f, 3.2f), trainM, root);
                    Prim(PrimitiveType.Cube, "SafetyLineN", new Vector3(o.X, 0.01f, o.Z + o.HZ + 0.4f), new Vector3(o.HX * 2f, 0.03f, 0.25f), stripeM, root);
                    Prim(PrimitiveType.Cube, "SafetyLineS", new Vector3(o.X, 0.01f, o.Z - o.HZ - 0.4f), new Vector3(o.HX * 2f, 0.03f, 0.25f), stripeM, root);
                    break;
                case "pillar":
                    Prim(PrimitiveType.Cylinder, "Pillar", new Vector3(o.X, 2f, o.Z), new Vector3(o.R * 2f, 2f, o.R * 2f), pillarM, root); break;
                case "kiosk":
                    Prim(PrimitiveType.Cube, "Kiosk", new Vector3(o.X, 1.25f, o.Z), new Vector3(o.HX * 2f, 2.5f, o.HZ * 2f), kioskM, root); break;
            }
        }
        // two public-address speakers mounted on pillars (they are the 3D audio sources)
        Prim(PrimitiveType.Cube, "Speaker1", new Vector3(-8f, 4.3f, 10f), new Vector3(0.7f, 0.5f, 0.7f), pitM, root);
        Prim(PrimitiveType.Cube, "Speaker2", new Vector3(8f, 4.3f, -10f), new Vector3(0.7f, 0.5f, 0.7f), pitM, root);

        for (int e = 0; e < CrowdSim.ExitCount; e++)
        {
            float ex = CrowdSim.ExitX[e]; int sg = CrowdSim.ExitSign[e];
            float zWall = sg * (CrowdSim.HalfZ + CrowdSim.Wall / 2f);
            Prim(PrimitiveType.Cube, "ExitMarker" + CrowdSim.ExitName[e], new Vector3(ex, 0.02f, sg * (CrowdSim.HalfZ - 1.5f)), new Vector3(4f, 0.03f, 3f), exitM, root);
            Prim(PrimitiveType.Cube, "ExitSign" + CrowdSim.ExitName[e], new Vector3(ex, 3.4f, zWall), new Vector3(4f, 0.6f, 0.3f), exitM, root);
            barrierGO[e] = Prim(PrimitiveType.Cube, "Barrier" + CrowdSim.ExitName[e], new Vector3(ex, 1.1f, zWall), new Vector3(4f, 2.2f, 1f), barrierM, root);
            barrierGO[e].SetActive(false);
        }
    }

    // ---------------- crowd (re)creation ----------------
    void ResetCrowd()
    {
        for (int i = crowd.childCount - 1; i >= 0; i--) Destroy(crowd.GetChild(i).gameObject);
        agentTr.Clear(); agentRend.Clear(); agentMF.Clear(); agentMat.Clear(); agentLow.Clear(); walkPhase.Clear();
        sim.Spawn(agentTarget);
        foreach (var a in sim.Agents)
        {
            var g = Prim(PrimitiveType.Capsule, "Agent" + a.Id, new Vector3(a.X, 0.85f, a.Z), new Vector3(a.R * 2f, 0.85f, a.R * 2f), stateMats[0], crowd);
            agentTr.Add(g.transform); agentRend.Add(g.GetComponent<Renderer>()); agentMF.Add(g.GetComponent<MeshFilter>());
            agentMat.Add(0); agentLow.Add(false); walkPhase.Add(a.Id * 0.7f);
        }
        if (capsuleMesh == null && agentMF.Count > 0)
        {
            capsuleMesh = agentMF[0].sharedMesh;
            capsuleTris = (int)(capsuleMesh.GetIndexCount(0) / 3);
        }
        viewIdx = 0; acc = 0f; lookYaw = 0f; lookPitch = 0f;
        log.Clear(); nextLog = 0f; finalLogged = false; exportMsg = "";
        StopSirens();
    }

    // ---------------- audio: procedurally generated (no audio files needed) ----------------
    void SetupAudio()
    {
        int sr = 44100, n = sr * 2;
        var data = new float[n]; float phase = 0f;
        for (int i = 0; i < n; i++)
        {
            float t = i / (float)sr;
            float f = 800f + 300f * Mathf.Sin(2f * Mathf.PI * 1f * t);          // 500-1100 Hz wail, 1 Hz sweep
            phase += 2f * Mathf.PI * f / sr;
            data[i] = 0.25f * Mathf.Sin(phase);
        }
        var siren = AudioClip.Create("Siren", n, 1, sr, false); siren.SetData(data, 0);
        Vector3[] spk = { new Vector3(-8f, 4.3f, 10f), new Vector3(8f, 4.3f, -10f) };
        for (int i = 0; i < spk.Length; i++)                                      // 3D positional sources -> pans/attenuates with the listener
        {
            var g = new GameObject("SirenSpeaker" + i); g.transform.position = spk[i];
            var s = g.AddComponent<AudioSource>();
            s.clip = siren; s.loop = true; s.playOnAwake = false; s.volume = 0.6f;
            s.spatialBlend = 1f; s.rolloffMode = AudioRolloffMode.Linear; s.minDistance = 8f; s.maxDistance = 120f;
            sirens.Add(s);
        }
        // crowd murmur: low-passed noise, louder as the average panic rises
        int m = sr * 3; var nd = new float[m]; float lp = 0f; var rnd = new System.Random(3);
        for (int i = 0; i < m; i++)
        {
            float w = (float)(rnd.NextDouble() * 2.0 - 1.0);
            lp += 0.06f * (w - lp);
            nd[i] = Mathf.Clamp(lp * 3.5f * (0.65f + 0.35f * Mathf.Sin(i * 0.0004f)), -1f, 1f);
        }
        var clip = AudioClip.Create("Murmur", m, 1, sr, false); clip.SetData(nd, 0);
        murmur = gameObject.AddComponent<AudioSource>();
        murmur.clip = clip; murmur.loop = true; murmur.spatialBlend = 0f; murmur.volume = 0.05f; murmur.Play();
    }

    void StartAlert()
    {
        if (soundOn) foreach (var s in sirens) if (!s.isPlaying) s.Play();
#if (UNITY_ANDROID || UNITY_IOS) && !UNITY_EDITOR
        Handheld.Vibrate();                                                       // haptic channel on phones
#endif
    }

    void StopSirens() { foreach (var s in sirens) s.Stop(); }

    // =====================================================================
    void Update()
    {
        float dtReal = Time.unscaledDeltaTime;
        if (dtReal > 0f) { fps = Mathf.Lerp(fps, 1f / dtReal, 0.05f); frameMs = Mathf.Lerp(frameMs, dtReal * 1000f, 0.05f); }
        if (benchRequested) { benchRequested = false; RunBenchmark(); }

        // fixed-timestep simulation (50 Hz) with a cap so a slow machine never freezes
        int steps = 0;
        if (!paused)
        {
            acc += Time.deltaTime * speedMul;
            int maxSteps = 6 * speedMul;
            watch.Reset(); watch.Start();
            while (acc >= CrowdSim.Dt && steps < maxSteps) { sim.Step(CrowdSim.Dt); acc -= CrowdSim.Dt; steps++; }
            watch.Stop();
            if (steps == maxSteps) acc = 0f;
            simMs = Mathf.Lerp(simMs, (float)watch.Elapsed.TotalMilliseconds, 0.05f);
        }
        if (scenarioOn) RunScenario();
        RecordLog();

        // mirror simulation -> GameObjects (position, heading, walk bob, forward lean, colour = behaviour state)
        var list = sim.Agents;
        for (int i = 0; i < list.Count; i++)
        {
            var a = list[i]; var tr = agentTr[i];
            if (a.State == AgentState.Escaped) { if (tr.gameObject.activeSelf) tr.gameObject.SetActive(false); continue; }
            float spd = Mathf.Sqrt(a.VX * a.VX + a.VZ * a.VZ);
            walkPhase[i] += spd * Time.deltaTime * 6f;
            float bob = Mathf.Abs(Mathf.Sin(walkPhase[i])) * 0.05f * Mathf.Min(spd, 2f);
            tr.localPosition = new Vector3(a.X, 0.85f + bob, a.Z);
            Vector3 fwd = new Vector3(a.DirX, 0f, a.DirZ);
            if (fwd.sqrMagnitude > 1e-4f)
                tr.localRotation = Quaternion.LookRotation(fwd) * Quaternion.Euler(Mathf.Clamp(spd * 7f, 0f, 16f), 0f, 0f);
            int m = (int)a.State; if (m > 3) m = 3;
            if (agentMat[i] != m) { agentMat[i] = m; agentRend[i].sharedMaterial = stateMats[m]; }
        }
        if (Time.frameCount % 6 == 0) UpdateRenderStats();

        // audio: murmur follows the average panic, sirens stop once everyone is out
        bool done = sim.AlarmOn && list.Count > 0 && sim.EscapedCount == list.Count;
        float mv = !soundOn ? 0f : (done ? 0.02f : (sim.AlarmOn ? 0.15f + 0.7f * sim.AvgPanic : 0.05f));
        murmur.volume = Mathf.Lerp(murmur.volume, mv, 0.1f);
        if (done) StopSirens();
    }

    // LOD swap (capsule -> cube beyond a distance) and frustum-visibility count
    void UpdateRenderStats()
    {
        if (capsuleMesh == null) return;
        Vector3 cp = cam.transform.position; float lim = lodDist * lodDist; int tris = 0, vis = 0;
        GeometryUtility.CalculateFrustumPlanes(cam, planes);
        var list = sim.Agents;
        for (int i = 0; i < list.Count; i++)
        {
            var a = list[i]; if (a.State == AgentState.Escaped) continue;
            Vector3 d = new Vector3(a.X, 0.85f, a.Z) - cp;
            bool low = useLOD && d.sqrMagnitude > lim;
            if (low != agentLow[i]) { agentLow[i] = low; agentMF[i].sharedMesh = low ? cubeMesh : capsuleMesh; }
            tris += low ? cubeTris : capsuleTris;
            if (GeometryUtility.TestPlanesAABB(planes, agentRend[i].bounds)) vis++;
        }
        agentTris = tris; inView = vis;
    }

    void LateUpdate()
    {
        Vector3 pos; Quaternion rot; float fov = 50f;
        if (viewMode == 2 && EnsureViewAgent())
        {
            var a = sim.Agents[viewIdx];
            Vector3 f = new Vector3(a.DirX, 0f, a.DirZ);
            if (f.sqrMagnitude < 1e-4f) f = Vector3.forward;
            rot = Quaternion.AngleAxis(lookYaw, Vector3.up) * Quaternion.LookRotation(f.normalized, Vector3.up) * Quaternion.Euler(lookPitch, 0f, 0f);
            pos = new Vector3(a.X, 1.6f, a.Z); fov = stereo ? 90f : 75f;
            agentRend[viewIdx].enabled = false;               // hide the body we are looking out of
        }
        else
        {
            float pitch = viewMode == 0 ? 90f : 50f;
            rot = Quaternion.Euler(pitch, yaw, 0f);
            pos = rot * (Vector3.back * zoom);
        }
        Vector3 right = rot * Vector3.right;
        float half = stereo ? Ipd * 0.5f : 0f;
        cam.transform.SetPositionAndRotation(pos - right * half, rot);
        cam.fieldOfView = fov;
        cam.rect = stereo ? new Rect(0f, 0f, 0.5f, 1f) : new Rect(0f, 0f, 1f, 1f);
        camR.enabled = stereo;
        if (stereo)
        {
            camR.transform.SetPositionAndRotation(pos + right * half, rot);
            camR.fieldOfView = fov; camR.rect = new Rect(0.5f, 0f, 0.5f, 1f);
            camR.clearFlags = cam.clearFlags; camR.backgroundColor = cam.backgroundColor;
        }
    }

    bool EnsureViewAgent()
    {
        var list = sim.Agents;
        if (list.Count == 0) return false;
        if (viewIdx >= list.Count) viewIdx = 0;
        if (list[viewIdx].State == AgentState.Escaped) NextViewAgent();
        return list[viewIdx].State != AgentState.Escaped;
    }

    void NextViewAgent()
    {
        var list = sim.Agents; if (list.Count == 0) return;
        if (viewIdx < agentRend.Count) agentRend[viewIdx].enabled = true;
        for (int k = 1; k <= list.Count; k++)
        {
            int j = (viewIdx + k) % list.Count;
            if (list[j].State != AgentState.Escaped) { viewIdx = j; lookYaw = 0f; lookPitch = 0f; return; }
        }
    }

    void SetView(int m)
    {
        if (viewMode == 2 && m != 2 && viewIdx < agentRend.Count) agentRend[viewIdx].enabled = true;
        viewMode = m; lookYaw = 0f; lookPitch = 0f;
        if (m == 0) zoom = 50f; else if (m == 1) zoom = 60f;
    }

    void BlockExit(int e, bool b) { sim.SetBlocked(e, b); barrierGO[e].SetActive(b); }

    // =====================================================================
    //  One-click scripted scenario (for a reliable live demo)
    // =====================================================================
    void StartScenario()
    {
        agentTarget = 300; speedMul = 1; paused = false;
        for (int e = 0; e < CrowdSim.ExitCount; e++) BlockExit(e, false);
        ResetCrowd();
        for (int i = 0; i < scenFired.Length; i++) scenFired[i] = false;
        scenarioLog = "Scenario started: 300 people waiting.\n"; scenarioOn = true;
    }

    void RunScenario()
    {
        float t = sim.SimTime;
        if (!scenFired[0] && t >= 3f) { scenFired[0] = true; BlockExit(0, true); scenarioLog += "t=3s  Exit A blocked (fire door jammed).\n"; }
        if (!scenFired[1] && t >= 5f) { scenFired[1] = true; sim.TriggerAlarm(); StartAlert(); scenarioLog += "t=5s  Fire alarm triggered.\n"; }
        if (!scenFired[2] && t >= 14f) { scenFired[2] = true; sim.SeedPanic(0.1f); scenarioLog += "t=14s Panic breaks out in 10% of the crowd.\n"; }
        if (!scenFired[3] && t >= 20f) { scenFired[3] = true; BlockExit(3, true); scenarioLog += "t=20s Exit D blocked as well.\n"; }
        if (sim.FinishTime >= 0f) { scenarioOn = false; scenarioLog += "Evacuation complete in " + sim.FinishTime.ToString("0.0") + " s.\n"; }
    }

    // =====================================================================
    //  Result logging (CSV for the report graphs) and benchmark
    // =====================================================================
    void RecordLog()
    {
        if (!sim.AlarmOn) return;
        float t = sim.EvacTime;
        bool finished = sim.FinishTime >= 0f;
        if ((t >= nextLog && !finished) || (finished && !finalLogged))
        {
            log.Add(F(t, "0.0") + "," + sim.EscapedCount + "," + (sim.Agents.Count - sim.EscapedCount) + ","
                    + F(sim.AvgPanic, "0.00") + "," + F(fps, "0") + "," + F(frameMs, "0.0"));
            nextLog += 1f; if (finished) finalLogged = true;
        }
    }

    void ExportResults()
    {
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("# agents=" + sim.Agents.Count + " spatial_hash=" + sim.UseSpatialHash
                      + " exit_A=" + sim.EscapedPerExit[0] + " B=" + sim.EscapedPerExit[1] + " C=" + sim.EscapedPerExit[2] + " D=" + sim.EscapedPerExit[3]);
        sb.AppendLine("evac_time_s,escaped,inside,avg_panic,fps,frame_ms");
        foreach (var row in log) sb.AppendLine(row);
        GUIUtility.systemCopyBuffer = sb.ToString();
        exportMsg = "Copied " + log.Count + " rows to clipboard.";
        try
        {
            string p = System.IO.Path.Combine(Application.persistentDataPath, "evac_results.csv");
            System.IO.File.WriteAllText(p, sb.ToString());
            exportMsg += " Saved: " + p;
        }
        catch (System.Exception) { }
        Debug.Log(sb.ToString());
    }

    // Headless timing of the simulation step with / without the spatial hash
    void RunBenchmark()
    {
        var sb = new System.Text.StringBuilder("Sim cost per step (ms), 50 Hz fixed step\nAgents   Hash    Brute-force\n");
        foreach (int n in new[] { 100, 300, 500, 800 })
        {
            float[] r = new float[2];
            for (int mode = 0; mode < 2; mode++)
            {
                var s = new CrowdSim(7); s.UseSpatialHash = (mode == 0);
                s.Spawn(n); s.TriggerAlarm();
                for (int i = 0; i < 60; i++) s.Step(CrowdSim.Dt);          // warm-up
                var w = Stopwatch.StartNew();
                for (int i = 0; i < 150; i++) s.Step(CrowdSim.Dt);
                w.Stop(); r[mode] = (float)w.Elapsed.TotalMilliseconds / 150f;
            }
            sb.AppendLine(n.ToString().PadRight(9) + r[0].ToString("0.00").PadRight(8) + r[1].ToString("0.00"));
        }
        benchText = sb.ToString();
        Debug.Log(benchText);
    }

    // =====================================================================
    //  UI (IMGUI: works with mouse and touch)
    // =====================================================================
    void OnGUI()
    {
        if (sim == null || cam == null) return;
        float s = Mathf.Clamp(Screen.height / 640f, 1f, 3f);
        GUI.matrix = Matrix4x4.Scale(new Vector3(s, s, 1f));
        float sh = Screen.height / s;
        GUI.skin.label.wordWrap = true; GUI.skin.label.richText = true; GUI.skin.toggle.wordWrap = true;
        if (labelStyle == null)
            labelStyle = new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleCenter, fontStyle = FontStyle.Bold, fontSize = 14, wordWrap = false };

        Rect menuRect = showUI ? new Rect(6, 6, 275, sh - 12) : new Rect(6, 6, 70, 28);
        HandlePointer(menuRect);
        DrawExitLabels(s);
        if (stereo) GUI.Box(new Rect(Screen.width / s * 0.5f - 1f, 0f, 2f, sh), GUIContent.none);

        if (!showUI) { if (GUI.Button(menuRect, "Menu")) showUI = true; return; }

        float budget = 1000f / 90f;
        string frameCol = frameMs > budget ? "#FF5A5A" : "#6CFF8A";

        GUILayout.BeginArea(menuRect, GUI.skin.box);
        scroll = GUILayout.BeginScrollView(scroll);
        GUILayout.Label("<b>METRO STATION EVACUATION</b>\nCS4104 CG & VR - Case Study 4");
        GUILayout.Label("FPS <b>" + fps.ToString("0") + "</b>  frame <color=" + frameCol + "><b>" + frameMs.ToString("0.0") + " ms</b></color> (VR budget @90 Hz = 11.1 ms)\n"
                        + "Simulation <b>" + simMs.ToString("0.00") + " ms</b>/frame");
        GUILayout.Label("Inside <b>" + (sim.Agents.Count - sim.EscapedCount) + "</b> / " + sim.Agents.Count
                        + "   Escaped <b>" + sim.EscapedCount + "</b>\n"
                        + "Evac time <b>" + sim.EvacTime.ToString("0.0") + " s</b>   Avg panic <b>" + sim.AvgPanic.ToString("0.00") + "</b>\n"
                        + "Out per exit  A:" + sim.EscapedPerExit[0] + "  B:" + sim.EscapedPerExit[1]
                        + "  C:" + sim.EscapedPerExit[2] + "  D:" + sim.EscapedPerExit[3]);

        if (GUILayout.Button("RUN SCENARIO (automatic demo)", GUILayout.Height(30))) StartScenario();
        if (scenarioLog.Length > 0) GUILayout.Label("<size=11>" + scenarioLog + "</size>");

        GUILayout.BeginHorizontal();
        if (GUILayout.Button(sim.AlarmOn ? "Alarm ON" : "TRIGGER ALARM", GUILayout.Height(30))) { sim.TriggerAlarm(); StartAlert(); }
        if (GUILayout.Button("Spread Panic", GUILayout.Height(30))) { sim.SeedPanic(0.1f); StartAlert(); }
        GUILayout.EndHorizontal();

        GUILayout.Label("Block exits (barrier appears):");
        GUILayout.BeginHorizontal();
        for (int e = 0; e < CrowdSim.ExitCount; e++)
        {
            bool b = GUILayout.Toggle(sim.IsBlocked(e), CrowdSim.ExitName[e], GUI.skin.button, GUILayout.Height(26));
            if (b != sim.IsBlocked(e)) BlockExit(e, b);
        }
        GUILayout.EndHorizontal();

        GUILayout.Label("Crowd size: <b>" + agentTarget + "</b>");
        agentTarget = Mathf.RoundToInt(GUILayout.HorizontalSlider(agentTarget, 20, 800));
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Reset / Respawn", GUILayout.Height(26))) { scenarioOn = false; scenarioLog = ""; ResetCrowd(); }
        paused = GUILayout.Toggle(paused, "Pause", GUI.skin.button, GUILayout.Height(26));
        GUILayout.EndHorizontal();

        GUILayout.Label("Simulation speed:");
        GUILayout.BeginHorizontal();
        int[] sp = { 1, 2, 4 };
        foreach (int v in sp) if (GUILayout.Toggle(speedMul == v, v + "x", GUI.skin.button)) speedMul = v;
        GUILayout.EndHorizontal();

        GUILayout.Label("Camera (drag = look/orbit, wheel = zoom):");
        GUILayout.BeginHorizontal();
        if (GUILayout.Toggle(viewMode == 0, "Top", GUI.skin.button)) { if (viewMode != 0) SetView(0); }
        if (GUILayout.Toggle(viewMode == 1, "Angled", GUI.skin.button)) { if (viewMode != 1) SetView(1); }
        if (GUILayout.Toggle(viewMode == 2, "Agent eyes", GUI.skin.button)) { if (viewMode != 2) SetView(2); }
        GUILayout.EndHorizontal();
        if (viewMode == 2) { if (GUILayout.Button("Next agent")) NextViewAgent(); }
        else zoom = GUILayout.HorizontalSlider(zoom, 25f, 90f);

        stereo = GUILayout.Toggle(stereo, "Stereo split-screen (VR headset view, IPD 64 mm)");
        bool snd = GUILayout.Toggle(soundOn, "Sound (3D siren + crowd murmur)");
        if (snd != soundOn) { soundOn = snd; if (!snd) StopSirens(); }

        GUILayout.Label("<b>Optimisation</b>");
        sim.UseSpatialHash = GUILayout.Toggle(sim.UseSpatialHash, "Spatial hash neighbour search (off = brute force O(N^2))");
        useLOD = GUILayout.Toggle(useLOD, "Level of detail: far agents drawn as boxes");
        if (useLOD) { GUILayout.Label("LOD distance: " + lodDist.ToString("0") + " m"); lodDist = GUILayout.HorizontalSlider(lodDist, 10f, 80f); }
        GUILayout.Label("In camera frustum: <b>" + inView + "</b> / " + (sim.Agents.Count - sim.EscapedCount)
                        + "\nAgent triangles: <b>" + (agentTris / 1000f).ToString("0.0") + " k</b>");

        if (GUILayout.Button("Run benchmark (few seconds)")) { benchText = "Running..."; benchRequested = true; }
        if (benchText.Length > 0) GUILayout.Label("<size=11>" + benchText + "</size>");
        if (GUILayout.Button("Copy results as CSV (for report)")) ExportResults();
        if (exportMsg.Length > 0) GUILayout.Label("<size=10>" + exportMsg + "</size>");

        GUILayout.Label("<color=#73B3FF>blue</color> waiting  <color=#FFD933>yellow</color> alerted  <color=#40D959>green</color> evacuating  <color=#F22626>red</color> panicked");
        if (GUILayout.Button("Hide menu")) showUI = false;
        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    // Drag to look (agent view) / orbit (overview); mouse wheel zooms. IMGUI events work for mouse and touch.
    void HandlePointer(Rect menuRect)
    {
        Event ev = Event.current;
        if (ev == null || menuRect.Contains(ev.mousePosition)) return;
        if (ev.type == EventType.MouseDrag)
        {
            if (viewMode == 2) { lookYaw -= ev.delta.x * 0.25f; lookPitch = Mathf.Clamp(lookPitch - ev.delta.y * 0.25f, -60f, 60f); }
            else yaw -= ev.delta.x * 0.3f;
        }
        else if (ev.type == EventType.ScrollWheel && viewMode != 2)
            zoom = Mathf.Clamp(zoom + ev.delta.y * 2f, 25f, 90f);
    }

    void DrawExitLabels(float s)
    {
        for (int e = 0; e < CrowdSim.ExitCount; e++)
        {
            Vector3 wp = new Vector3(CrowdSim.ExitX[e], 4.2f, CrowdSim.ExitSign[e] * (CrowdSim.HalfZ + CrowdSim.Wall / 2f));
            Vector3 sp = cam.WorldToScreenPoint(wp);
            if (sp.z <= 0f) continue;
            bool bl = sim.IsBlocked(e);
            GUI.color = bl ? new Color(1f, 0.3f, 0.3f) : new Color(0.4f, 1f, 0.5f);
            GUI.Label(new Rect(sp.x / s - 60f, (Screen.height - sp.y) / s - 12f, 120f, 24f),
                      "EXIT " + CrowdSim.ExitName[e] + (bl ? " (BLOCKED)" : ""), labelStyle);
        }
        GUI.color = Color.white;
    }
}
