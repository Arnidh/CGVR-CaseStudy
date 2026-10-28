// CS4104 - Computer Graphics & Virtual Reality | Case Study 4: Behavioural Simulation & Crowd Dynamics
// CrowdSimulation.cs  -  the "brain" of the metro-station evacuation (no rendering code in here).
//
// Techniques used (all are discussed in the report):
//   * Intelligent agents with a finite-state machine: Waiting -> Alerted -> Evacuating -> Panicked -> Escaped
//   * Per-exit navigation (distance) fields on a grid, computed once  (SPFA / Dijkstra-style wavefront)
//   * Social-force model (Helbing) for local collision avoidance between agents and walls
//   * Panic contagion + herding + tunnel vision, exit-blocked knowledge that spreads by word of mouth
//   * Uniform spatial hash so neighbour search is O(N) instead of O(N^2)
using System;
using System.Collections.Generic;
using UnityEngine;

namespace MetroEvac
{
    public enum AgentState { Waiting = 0, Alerted = 1, Evacuating = 2, Panicked = 3, Escaped = 4 }

    /// <summary>Static or dynamic obstacle in the XZ plane: axis-aligned box or circle.</summary>
    public sealed class Obstacle
    {
        public bool Circle;
        public string Kind;
        public float X, Z, HX, HZ, R;   // box: centre + half extents, circle: centre + radius

        public static Obstacle Box(string kind, float x, float z, float hx, float hz)
        { return new Obstacle { Kind = kind, X = x, Z = z, HX = hx, HZ = hz }; }

        public static Obstacle Round(string kind, float x, float z, float r)
        { return new Obstacle { Kind = kind, Circle = true, X = x, Z = z, R = r }; }

        /// <summary>Distance from a point to the obstacle surface (0 if inside).</summary>
        public float Distance(float px, float pz)
        {
            if (Circle)
            {
                float d = Mathf.Sqrt((px - X) * (px - X) + (pz - Z) * (pz - Z)) - R;
                return d > 0f ? d : 0f;
            }
            float dx = Mathf.Max(Mathf.Abs(px - X) - HX, 0f);
            float dz = Mathf.Max(Mathf.Abs(pz - Z) - HZ, 0f);
            return Mathf.Sqrt(dx * dx + dz * dz);
        }
    }

    public sealed class Agent
    {
        public int Id;
        public float X, Z, VX, VZ, R;
        public float DirX = 0f, DirZ = 1f;            // smoothed heading (used by the first-person camera)
        // personality
        public float Fear, React, BaseSpeed, Herd;
        public int Familiar;                           // exit this person knows best
        // state
        public AgentState State;
        public float Panic;                            // 0..1
        public float Alert;                            // reaction-time countdown
        public int Exit = -1;                          // exit currently targeted
        public bool[] Known = new bool[CrowdSim.ExitCount];   // "I know this exit is blocked"
        public float EvalT;
        public float WX, WZ, WT;                       // idle wander
        // perception (refreshed every 5 simulation steps)
        public int NbCount; public float NbPanic, NbVX, NbVZ;
        public int Cell;
        public float EscapedAt = -1f;
    }

    public sealed class CrowdSim
    {
        // ---------------- station geometry (metres) ----------------
        public const float HalfX = 30f, HalfZ = 20f, Wall = 1f, GapHalf = 2f;
        public const int ExitCount = 4;
        public static readonly float[] ExitX = { -20f, 20f, -20f, 20f };
        public static readonly int[] ExitSign = { 1, 1, -1, -1 };          // +1 = north wall, -1 = south wall
        public static readonly string[] ExitName = { "A", "B", "C", "D" };

        // navigation grid
        const float Cell = 0.5f, OriginX = -34f, OriginZ = -24f;
        const int NX = 136, NZ = 96;
        const float INF = 1e9f;
        public const float Dt = 0.02f;                 // fixed simulation step = 50 Hz

        // spatial hash
        const float HCS = 1.5f, HOX = -36f, HOZ = -26f;
        const int HW = 48, HH = 36;

        // ---------------- public state ----------------
        public bool UseSpatialHash = true;
        public bool AlarmOn { get; private set; }
        public float SimTime { get; private set; }
        public int EscapedCount { get; private set; }
        public float AvgPanic { get; private set; }
        public float FinishTime { get; private set; }
        public readonly int[] EscapedPerExit = new int[ExitCount];
        public readonly List<Agent> Agents = new List<Agent>();
        public IList<Obstacle> StaticObstacles { get { return statics; } }
        public float EvacTime { get { return !AlarmOn ? 0f : (FinishTime >= 0f ? FinishTime : SimTime - alarmTime); } }

        // ---------------- private ----------------
        sealed class Field { public float[] d = new float[NX * NZ], gx = new float[NX * NZ], gz = new float[NX * NZ]; }

        readonly List<Obstacle> statics = new List<Obstacle>();
        readonly List<Obstacle> all = new List<Obstacle>();
        readonly Obstacle[] barriers = new Obstacle[ExitCount];
        readonly bool[] blocked = new bool[ExitCount];
        readonly Field[] fields = new Field[ExitCount];
        readonly List<Agent> active = new List<Agent>();
        readonly List<Agent> nb = new List<Agent>();
        readonly int[] queueCount = new int[ExitCount];
        readonly int[] cellStart = new int[HW * HH + 1];
        readonly int[] fillPos = new int[HW * HH];
        Agent[] sorted = new Agent[1024];
        System.Random rng;
        int step;
        float alarmTime;

        static readonly int[] DI = { 1, -1, 0, 0, 1, 1, -1, -1 };
        static readonly int[] DJ = { 0, 0, 1, -1, 1, -1, 1, -1 };
        static readonly float[] DW = { 1f, 1f, 1f, 1f, 1.4142f, 1.4142f, 1.4142f, 1.4142f };

        public CrowdSim(int seed)
        {
            rng = new System.Random(seed);
            BuildStation();
            for (int e = 0; e < ExitCount; e++)
                barriers[e] = Obstacle.Box("barrier", ExitX[e], ExitSign[e] * (HalfZ + Wall / 2f), GapHalf, Wall / 2f);
            RebuildObstacleList();
            for (int e = 0; e < ExitCount; e++) fields[e] = BuildField(e);
            FinishTime = -1f;
        }

        // =====================================================================
        //  Setup
        // =====================================================================
        void BuildStation()
        {
            float[,] segs = { { -31f, -22f }, { -18f, 18f }, { 22f, 31f } };   // north/south walls with 4 m exit gaps
            foreach (int s in new[] { 1, -1 })
            {
                float zc = s * (HalfZ + Wall / 2f);
                for (int i = 0; i < 3; i++)
                    statics.Add(Obstacle.Box("wall", (segs[i, 0] + segs[i, 1]) / 2f, zc, (segs[i, 1] - segs[i, 0]) / 2f, Wall / 2f));
                statics.Add(Obstacle.Box("wall", s * (HalfX + Wall / 2f), 0f, Wall / 2f, HalfZ + Wall));   // west / east walls
            }
            statics.Add(Obstacle.Box("pit", 0f, 0f, 14f, 3.5f));            // track pit with parked train
            foreach (float x in new[] { -24f, -8f, 8f, 24f })
                foreach (float z in new[] { -10f, 10f })
                    statics.Add(Obstacle.Round("pillar", x, z, 0.6f));
            statics.Add(Obstacle.Box("kiosk", -8f, 15f, 2f, 1f));
            statics.Add(Obstacle.Box("kiosk", 8f, -15f, 2f, 1f));
        }

        void RebuildObstacleList()
        {
            all.Clear();
            all.AddRange(statics);
            for (int e = 0; e < ExitCount; e++) if (blocked[e]) all.Add(barriers[e]);
        }

        float Clearance(float px, float pz)
        {
            float m = INF;
            for (int i = 0; i < statics.Count; i++) { float d = statics[i].Distance(px, pz); if (d < m) m = d; }
            return m;
        }

        // Navigation field for one exit: walking distance (m) from every free cell to that exit.
        Field BuildField(int e)
        {
            var f = new Field();
            int n = NX * NZ;
            var free = new bool[n];
            float ex = ExitX[e]; int sg = ExitSign[e];
            for (int j = 0; j < NZ; j++)
                for (int i = 0; i < NX; i++)
                {
                    float px = OriginX + (i + 0.5f) * Cell, pz = OriginZ + (j + 0.5f) * Cell;
                    bool outside = Mathf.Abs(pz) > HalfZ + Wall || Mathf.Abs(px) > HalfX + Wall;
                    bool own = Mathf.Abs(px - ex) <= 1.5f && sg * pz > HalfZ + Wall && sg * pz <= HalfZ + Wall + 1.5f;
                    free[j * NX + i] = Clearance(px, pz) >= 0.55f && (!outside || own);   // outside is NOT walkable except own exit strip
                    f.d[j * NX + i] = INF;
                }
            var q = new Queue<int>(); var inq = new bool[n];
            for (int j = 0; j < NZ; j++)
                for (int i = 0; i < NX; i++)
                {
                    float px = OriginX + (i + 0.5f) * Cell, pz = OriginZ + (j + 0.5f) * Cell;
                    if (Mathf.Abs(px - ex) <= 1.5f && sg * pz >= HalfZ + Wall + 0.25f && sg * pz <= HalfZ + Wall + 1.5f && free[j * NX + i])
                    { f.d[j * NX + i] = 0f; q.Enqueue(j * NX + i); inq[j * NX + i] = true; }
                }
            while (q.Count > 0)
            {
                int c = q.Dequeue(); inq[c] = false;
                int ci = c % NX, cj = c / NX;
                for (int k = 0; k < 8; k++)
                {
                    int a = ci + DI[k], b = cj + DJ[k];
                    if (a < 0 || b < 0 || a >= NX || b >= NZ) continue;
                    int nn = b * NX + a;
                    if (!free[nn]) continue;
                    if (DI[k] != 0 && DJ[k] != 0 && !(free[cj * NX + a] && free[b * NX + ci])) continue;   // no corner cutting
                    float nd = f.d[c] + DW[k] * Cell;
                    if (nd < f.d[nn]) { f.d[nn] = nd; if (!inq[nn]) { inq[nn] = true; q.Enqueue(nn); } }
                }
            }
            // unit gradient (direction of steepest descent) per cell
            for (int j = 0; j < NZ; j++)
                for (int i = 0; i < NX; i++)
                {
                    int c = j * NX + i;
                    if (f.d[c] >= INF) continue;
                    float cv = f.d[c];
                    float gx = Get(f.d, i - 1, j, cv) - Get(f.d, i + 1, j, cv);
                    float gz = Get(f.d, i, j - 1, cv) - Get(f.d, i, j + 1, cv);
                    float l = Mathf.Sqrt(gx * gx + gz * gz);
                    if (l > 1e-6f) { f.gx[c] = gx / l; f.gz[c] = gz / l; }
                }
            return f;
        }

        static float Get(float[] d, int i, int j, float cv)
        {
            if (i < 0 || j < 0 || i >= NX || j >= NZ) return cv + 1f;
            float v = d[j * NX + i];
            return v >= INF ? cv + 1f : v;
        }

        // Bilinear sample of the navigation field: returns unit direction toward the exit and remaining distance.
        void FieldAt(Field f, float px, float pz, out float dx, out float dz, out float dist)
        {
            float fx = (px - OriginX) / Cell - 0.5f, fz = (pz - OriginZ) / Cell - 0.5f;
            int i0 = Mathf.FloorToInt(fx), j0 = Mathf.FloorToInt(fz);
            float tx = fx - i0, tz = fz - j0;
            float sx = 0f, sz = 0f, dv = 0f, ws = 0f;
            for (int k = 0; k < 4; k++)
            {
                int a = i0 + (k & 1), b = j0 + (k >> 1);
                float w = ((k & 1) == 0 ? 1f - tx : tx) * ((k >> 1) == 0 ? 1f - tz : tz);
                if (a < 0 || b < 0 || a >= NX || b >= NZ) continue;
                int c = b * NX + a;
                if (f.d[c] >= INF) continue;
                sx += f.gx[c] * w; sz += f.gz[c] * w; dv += f.d[c] * w; ws += w;
            }
            float l = Mathf.Sqrt(sx * sx + sz * sz);
            if (l > 1e-6f && ws > 1e-6f) { dx = sx / l; dz = sz / l; dist = dv / ws; return; }
            // squeezed against a wall/barrier: steer toward the nearest valid cell
            int ci = Mathf.FloorToInt((px - OriginX) / Cell), cj = Mathf.FloorToInt((pz - OriginZ) / Cell);
            float bd = INF, bx = 0f, bz = 0f; bool found = false;
            for (int r = 1; r < 6 && !found; r++)
                for (int a = ci - r; a <= ci + r; a++)
                    for (int b = cj - r; b <= cj + r; b++)
                    {
                        if (a < 0 || b < 0 || a >= NX || b >= NZ || f.d[b * NX + a] >= INF) continue;
                        float cx = OriginX + (a + 0.5f) * Cell, cz = OriginZ + (b + 0.5f) * Cell;
                        float dd = (cx - px) * (cx - px) + (cz - pz) * (cz - pz);
                        if (dd < bd) { bd = dd; bx = cx; bz = cz; found = true; }
                    }
            dist = INF;
            if (found && bd > 1e-10f) { float m = Mathf.Sqrt(bd); dx = (bx - px) / m; dz = (bz - pz) / m; }
            else { dx = 0f; dz = 0f; }
        }

        // =====================================================================
        //  Public controls
        // =====================================================================
        float U(float a, float b) { return a + (b - a) * (float)rng.NextDouble(); }
        float Gauss()
        {
            double u1 = 1.0 - rng.NextDouble(), u2 = rng.NextDouble();
            return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
        }

        public void Spawn(int n)
        {
            Agents.Clear(); EscapedCount = 0; SimTime = 0f; step = 0; AlarmOn = false; FinishTime = -1f; AvgPanic = 0f;
            Array.Clear(EscapedPerExit, 0, ExitCount);
            int tries = 0;
            while (Agents.Count < n && tries < 400000)
            {
                tries++;
                float x = U(-28.5f, 28.5f), z = U(-18.5f, 18.5f);
                if (Clearance(x, z) < 0.7f) continue;
                bool clash = false;
                for (int i = 0; i < Agents.Count; i++)
                { float dx = Agents[i].X - x, dz = Agents[i].Z - z; if (dx * dx + dz * dz < 0.55f * 0.55f) { clash = true; break; } }
                if (clash) continue;
                var a = new Agent();
                a.Id = Agents.Count; a.X = x; a.Z = z; a.R = U(0.22f, 0.28f);
                a.Fear = U(0f, 1f); a.React = U(0.3f, 3.5f);
                a.BaseSpeed = Mathf.Clamp(1.3f + 0.15f * Gauss(), 1f, 1.6f);
                a.Herd = U(0.2f, 1f); a.Familiar = rng.Next(ExitCount);
                float ang = U(0f, 6.2832f); a.WX = Mathf.Cos(ang); a.WZ = Mathf.Sin(ang); a.WT = U(1f, 4f);
                a.EvalT = U(0f, 1f); a.State = AgentState.Waiting;
                for (int e = 0; e < ExitCount; e++) a.Known[e] = false;
                Agents.Add(a);
            }
        }

        public void TriggerAlarm() { if (AlarmOn) return; AlarmOn = true; alarmTime = SimTime; }

        /// <summary>Make a fraction of the crowd panic at once (e.g. a loud bang); contagion does the rest.</summary>
        public void SeedPanic(float fraction)
        {
            TriggerAlarm();
            for (int i = 0; i < Agents.Count; i++)
                if (Agents[i].State != AgentState.Escaped && rng.NextDouble() < fraction) Agents[i].Panic = 1f;
        }

        public bool IsBlocked(int e) { return blocked[e]; }

        public void SetBlocked(int e, bool b)
        {
            if (blocked[e] == b) return;
            blocked[e] = b;
            if (!b) for (int i = 0; i < Agents.Count; i++) Agents[i].Known[e] = false;   // "exit re-opened" announcement
            RebuildObstacleList();
        }

        // =====================================================================
        //  Simulation step (fixed dt)
        // =====================================================================
        public void Step(float dt)
        {
            step++; SimTime += dt;
            active.Clear();
            for (int i = 0; i < Agents.Count; i++) if (Agents[i].State != AgentState.Escaped) active.Add(Agents[i]);
            if (UseSpatialHash) BuildHash();
            if (step % 25 == 0) UpdateQueues();
            for (int i = 0; i < active.Count; i++) if ((step + active[i].Id) % 5 == 0) Perceive(active[i]);
            for (int i = 0; i < active.Count; i++) Think(active[i], dt);
            if (step % 10 == 0)
            {
                float s = 0f; int c = 0;
                for (int i = 0; i < active.Count; i++) if (active[i].State != AgentState.Escaped) { s += active[i].Panic; c++; }
                AvgPanic = c > 0 ? s / c : 0f;
            }
        }

        // ---------- uniform spatial hash (counting sort) ----------
        int HashCell(float x, float z)
        {
            int cx = Mathf.Clamp(Mathf.FloorToInt((x - HOX) / HCS), 0, HW - 1);
            int cz = Mathf.Clamp(Mathf.FloorToInt((z - HOZ) / HCS), 0, HH - 1);
            return cz * HW + cx;
        }

        void BuildHash()
        {
            Array.Clear(cellStart, 0, cellStart.Length);
            if (sorted.Length < active.Count) sorted = new Agent[active.Count * 2];
            for (int i = 0; i < active.Count; i++) { var a = active[i]; a.Cell = HashCell(a.X, a.Z); cellStart[a.Cell + 1]++; }
            for (int i = 0; i < HW * HH; i++) cellStart[i + 1] += cellStart[i];
            Array.Copy(cellStart, fillPos, HW * HH);
            for (int i = 0; i < active.Count; i++) { var a = active[i]; sorted[fillPos[a.Cell]++] = a; }
        }

        // Fills 'nb' with agents within 'range' of a. Uses the hash (O(1) per agent) or brute force (O(N)).
        void Query(Agent a, float range)
        {
            nb.Clear();
            float r2 = range * range;
            if (!UseSpatialHash)
            {
                for (int i = 0; i < active.Count; i++)
                {
                    var b = active[i]; if (b == a) continue;
                    float dx = a.X - b.X, dz = a.Z - b.Z;
                    if (dx * dx + dz * dz <= r2) nb.Add(b);
                }
                return;
            }
            int cx = Mathf.Clamp(Mathf.FloorToInt((a.X - HOX) / HCS), 0, HW - 1);
            int cz = Mathf.Clamp(Mathf.FloorToInt((a.Z - HOZ) / HCS), 0, HH - 1);
            int k = Mathf.CeilToInt(range / HCS);
            for (int j = Mathf.Max(0, cz - k); j <= Mathf.Min(HH - 1, cz + k); j++)
                for (int i = Mathf.Max(0, cx - k); i <= Mathf.Min(HW - 1, cx + k); i++)
                {
                    int c = j * HW + i;
                    for (int idx = cellStart[c]; idx < cellStart[c + 1]; idx++)
                    {
                        var b = sorted[idx]; if (b == a) continue;
                        float dx = a.X - b.X, dz = a.Z - b.Z;
                        if (dx * dx + dz * dz <= r2) nb.Add(b);
                    }
                }
        }

        void UpdateQueues()
        {
            Array.Clear(queueCount, 0, ExitCount);
            for (int i = 0; i < active.Count; i++)
            {
                var a = active[i]; if (a.Exit < 0) continue;
                float dx = a.X - ExitX[a.Exit], dz = a.Z - ExitSign[a.Exit] * (HalfZ + Wall / 2f);
                if (dx * dx + dz * dz < 36f) queueCount[a.Exit]++;
            }
        }

        // What the agent "sees and hears": local density, neighbours' panic and heading, and word-of-mouth about blocked exits.
        void Perceive(Agent a)
        {
            Query(a, 3f);
            int n = 0, cnt = 0; float sp = 0f, svx = 0f, svz = 0f;
            for (int i = 0; i < nb.Count; i++)
            {
                var b = nb[i];
                float dx = a.X - b.X, dz = a.Z - b.Z; float d = Mathf.Sqrt(dx * dx + dz * dz);
                cnt++; sp += b.Panic; svx += b.VX; svz += b.VZ;
                if (d <= 1.2f) n++;
                if (d <= 2.5f)
                    for (int e = 0; e < ExitCount; e++)
                        if (blocked[e] && b.Known[e] && !a.Known[e] && rng.NextDouble() < 0.05)
                        { a.Known[e] = true; a.Panic = Mathf.Min(1f, a.Panic + 0.15f); a.EvalT = 0f; }
            }
            a.NbCount = n;
            a.NbPanic = cnt > 0 ? sp / cnt : 0f;
            a.NbVX = cnt > 0 ? svx / cnt : 0f;
            a.NbVZ = cnt > 0 ? svz / cnt : 0f;
        }

        int ChooseExit(Agent a)
        {
            int best = -1; float bc = INF;
            for (int e = 0; e < ExitCount; e++)
            {
                if (a.Known[e]) continue;
                float dx, dz, dv; FieldAt(fields[e], a.X, a.Z, out dx, out dz, out dv);
                float c = dv + (e == a.Familiar ? 0f : 8f) + 0.3f * queueCount[e];   // distance + familiarity bias + queue penalty
                if (c < bc) { bc = c; best = e; }
            }
            if (best < 0)   // believes every exit is blocked: just take the nearest
                for (int e = 0; e < ExitCount; e++)
                {
                    float dx, dz, dv; FieldAt(fields[e], a.X, a.Z, out dx, out dz, out dv);
                    if (dv < bc) { bc = dv; best = e; }
                }
            return best < 0 ? 0 : best;
        }

        // ---------- one agent: behaviour -> desired velocity -> forces -> integration -> collisions ----------
        void Think(Agent a, float dt)
        {
            // 1. behaviour state machine
            if (a.State == AgentState.Waiting && AlarmOn)
            { a.State = AgentState.Alerted; a.Alert = a.React; a.Panic = Mathf.Max(a.Panic, 0.25f * a.Fear); }
            if (a.State == AgentState.Alerted) { a.Alert -= dt; if (a.Alert <= 0f) a.State = AgentState.Evacuating; }
            if (a.State >= AgentState.Evacuating)
            {
                for (int e = 0; e < ExitCount; e++)       // sees a blocked exit when close enough
                    if (blocked[e] && !a.Known[e])
                    {
                        float dx = a.X - ExitX[e], dz = a.Z - ExitSign[e] * (HalfZ + Wall / 2f);
                        if (dx * dx + dz * dz < 36f)
                        { a.Known[e] = true; a.Panic = Mathf.Min(1f, a.Panic + 0.3f * (0.5f + a.Fear)); a.EvalT = 0f; }
                    }
                float ds = Mathf.Clamp01((a.NbCount - 3) / 6f);                        // crowd-density stress
                a.Panic += dt * (0.25f * ds + 0.6f * Mathf.Max(0f, a.NbPanic - a.Panic) * a.Herd);   // + contagion
                if (ds < 0.2f) a.Panic -= dt * 0.02f;                                   // slow calming
                a.Panic = Mathf.Clamp01(a.Panic);
                a.State = a.Panic > 0.65f ? AgentState.Panicked : AgentState.Evacuating;
                a.EvalT -= dt;
                if (a.Exit < 0 || a.EvalT <= 0f || a.Known[a.Exit])
                {
                    // panicked agents suffer "tunnel vision": they keep their exit unless they learn it is blocked
                    if (!(a.State == AgentState.Panicked && a.Exit >= 0 && !a.Known[a.Exit])) a.Exit = ChooseExit(a);
                    a.EvalT = 1f;
                }
            }

            // 2. desired velocity
            float dvx, dvz;
            if (a.State <= AgentState.Alerted)
            {
                a.WT -= dt;
                if (a.WT <= 0f) { float ang = U(0f, 6.2832f); a.WX = Mathf.Cos(ang); a.WZ = Mathf.Sin(ang); a.WT = U(2f, 6f); }
                dvx = a.WX * 0.25f; dvz = a.WZ * 0.25f;          // idle: slow aimless drifting
                if (a.State == AgentState.Alerted) { dvx = 0f; dvz = 0f; }   // reaction delay: stand and look around
            }
            else
            {
                float dx, dz, dd; FieldAt(fields[a.Exit], a.X, a.Z, out dx, out dz, out dd);
                float hh = 0.5f * a.Panic * a.Herd;                       // herding: follow the neighbours' average motion
                float l = Mathf.Sqrt(a.NbVX * a.NbVX + a.NbVZ * a.NbVZ);
                if (l > 0.2f && a.NbCount > 0)
                {
                    dx = (1f - hh) * dx + hh * a.NbVX / l; dz = (1f - hh) * dz + hh * a.NbVZ / l;
                    float m = Mathf.Sqrt(dx * dx + dz * dz);
                    if (m > 1e-6f) { dx /= m; dz /= m; }
                }
                if (a.Panic > 0.3f)                                       // erratic movement under panic
                {
                    float ang = U(-1f, 1f) * 0.35f * a.Panic, c = Mathf.Cos(ang), s = Mathf.Sin(ang);
                    float ndx = dx * c - dz * s, ndz = dx * s + dz * c; dx = ndx; dz = ndz;
                }
                float speed = a.BaseSpeed * (1f + 0.5f * a.Panic);
                dvx = dx * speed; dvz = dz * speed;
            }

            // 3. forces (social-force model)
            float tau = a.State == AgentState.Panicked ? 0.3f : 0.5f;
            float ax = (dvx - a.VX) / tau, az = (dvz - a.VZ) / tau;
            Query(a, 1.5f);
            for (int i = 0; i < nb.Count; i++)
            {
                var b = nb[i];
                float dx = a.X - b.X, dz = a.Z - b.Z, d = Mathf.Sqrt(dx * dx + dz * dz);
                if (d > 1.5f) continue;
                if (d < 1e-4f) { dx = U(-1f, 1f); dz = U(-1f, 1f); d = 1f; }
                float nx = dx / d, nz = dz / d, rs = a.R + b.R;
                float f = 4f * Mathf.Exp((rs - d) / 0.3f);               // psychological repulsion
                if (d < rs) f += 80f * (rs - d);                          // body contact
                ax += nx * f; az += nz * f;
            }
            for (int i = 0; i < all.Count; i++)                           // soft repulsion from walls / pillars / barriers
            {
                var o = all[i]; float d, nx, nz;
                if (!ClosestOnObstacle(o, a.X, a.Z, out d, out nx, out nz)) continue;
                if (d < 1f && d > 1e-4f) { float f = 3f * Mathf.Exp((a.R - d) / 0.25f); ax += nx * f; az += nz * f; }
            }
            float am = Mathf.Sqrt(ax * ax + az * az);
            if (am > 20f) { ax *= 20f / am; az *= 20f / am; }

            // 4. integrate
            a.VX += ax * dt; a.VZ += az * dt;
            float vm = Mathf.Sqrt(a.VX * a.VX + a.VZ * a.VZ);
            if (vm > 2.5f) { a.VX *= 2.5f / vm; a.VZ *= 2.5f / vm; vm = 2.5f; }
            a.X += a.VX * dt; a.Z += a.VZ * dt;
            if (vm > 0.2f) { a.DirX += (a.VX / vm - a.DirX) * 0.15f; a.DirZ += (a.VZ / vm - a.DirZ) * 0.15f; }

            // 5. hard collision resolution against walls / pillars / barriers
            for (int i = 0; i < all.Count; i++)
            {
                var o = all[i];
                if (o.Circle)
                {
                    float dx = a.X - o.X, dz = a.Z - o.Z, l = Mathf.Sqrt(dx * dx + dz * dz);
                    if (l < o.R + a.R)
                    {
                        if (l < 1e-5f) { dx = 1f; dz = 0f; l = 1f; }
                        float nx = dx / l, nz = dz / l, pen = o.R + a.R - l;
                        a.X += nx * pen; a.Z += nz * pen;
                        float vn = a.VX * nx + a.VZ * nz; if (vn < 0f) { a.VX -= vn * nx; a.VZ -= vn * nz; }
                    }
                }
                else
                {
                    float qx = Mathf.Clamp(a.X, o.X - o.HX, o.X + o.HX), qz = Mathf.Clamp(a.Z, o.Z - o.HZ, o.Z + o.HZ);
                    float dx = a.X - qx, dz = a.Z - qz, d = Mathf.Sqrt(dx * dx + dz * dz);
                    if (d < a.R)
                    {
                        float nx, nz, pen;
                        if (d > 1e-5f) { nx = dx / d; nz = dz / d; pen = a.R - d; }
                        else
                        {
                            float px = o.HX - Mathf.Abs(a.X - o.X), pz = o.HZ - Mathf.Abs(a.Z - o.Z);
                            if (px < pz) { nx = a.X >= o.X ? 1f : -1f; nz = 0f; pen = px + a.R; }
                            else { nx = 0f; nz = a.Z >= o.Z ? 1f : -1f; pen = pz + a.R; }
                        }
                        a.X += nx * pen; a.Z += nz * pen;
                        float vn = a.VX * nx + a.VZ * nz; if (vn < 0f) { a.VX -= vn * nx; a.VZ -= vn * nz; }
                    }
                }
            }

            // 6. escaped? (only possible by walking through an open gap)
            if (Mathf.Abs(a.Z) >= HalfZ + Wall - 0.1f)
            {
                a.State = AgentState.Escaped; a.EscapedAt = SimTime; EscapedCount++;
                EscapedPerExit[(a.X < 0f ? 0 : 1) + (a.Z > 0f ? 0 : 2)]++;
                if (EscapedCount == Agents.Count && AlarmOn && FinishTime < 0f) FinishTime = SimTime - alarmTime;
            }
        }

        // distance + outward unit normal from the obstacle surface to the point
        static bool ClosestOnObstacle(Obstacle o, float px, float pz, out float d, out float nx, out float nz)
        {
            if (o.Circle)
            {
                float dx = px - o.X, dz = pz - o.Z, l = Mathf.Sqrt(dx * dx + dz * dz);
                d = l - o.R; if (l < 1e-5f) { nx = 1f; nz = 0f; return false; }
                nx = dx / l; nz = dz / l; return true;
            }
            float qx = Mathf.Clamp(px, o.X - o.HX, o.X + o.HX), qz = Mathf.Clamp(pz, o.Z - o.HZ, o.Z + o.HZ);
            float ddx = px - qx, ddz = pz - qz; d = Mathf.Sqrt(ddx * ddx + ddz * ddz);
            if (d < 1e-5f) { nx = 0f; nz = 0f; return false; }
            nx = ddx / d; nz = ddz / d; return true;
        }
    }
}
