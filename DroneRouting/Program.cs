using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using ILOG.Concert;
using ILOG.CPLEX;

namespace DroneRouter
{
    class DroneRouter   
    {

        public enum NodeType {ServiceLoc, RSLoc };
        public class Instance
        {
            public int Id { get; set; }
            public double MaxBattery{ get; set; }
            public double Phi_BattConTravel { get; set; }
            public double Omega_BattConHover { get; set; } 
            public double Rho_TimeRecharge { get; set; } 
            public double Tbig { get; set; }
            public double Speed { get; set; }
            public List<Location> LocList { get; set; }

            public List<RechargeStopLoc> RSList { get; set; }
        }

        public class Location
        {
            public int IdLoc { get; set; }
            public double Xcoord { get; set; }
            public double Ycoord { get; set; }
            public double ServiceTime { get; set; }
        }

        public class RechargeStopLoc
        {
            public int IdRS { get; set; }
            public double Xcoord { get; set; }
            public double Ycoord { get; set; }
            public int InitNumDrones { get; set; }
        }

        public class Node
        {
            public int IdN { get; set; }
            public NodeType Type { get; set; }

        }
        
        public class Arc
        {
            public Node Nodei { get; set; }
            public Node Nodej { get; set; }
            public double TravelTime { get; set; }
            public double Servicei { get; set; }
        }

        public class Route
        {
            public int IdDrone { get; set; }
            public List<Arc> DroneRoute { get; set; }
            public double TotalTime { get; set; } 
        }

        public class X_index: IEquatable<X_index>
        {
            public int i { get; set; }
            public int j { get; set; }

             public X_index(int ii, int jj)
            {
                i = ii; j = jj;
            }

            public bool Equals(X_index other)
            {
                if (other == null)
                    return false;

                if (this.i == other.i && this.j == other.j)
                    return true;

                return false;
            }

            public override int GetHashCode()
            {
                int hash = 17;
                hash = hash * 23 + i.GetHashCode();
                hash = hash * 23 + j.GetHashCode();
                return hash;
            }
        }

        public class Z_index: IEquatable<Z_index>
        {
            public int i { get; set; }
            public int j { get; set; }
            public int s { get; set; }

            public Z_index(int ii, int jj, int ss)
            {
                i = ii; j = jj; s = ss;
            }

            public bool Equals(Z_index other)
            {
                if (other == null)
                    return false;

                if (this.i == other.i && this.j == other.j && this.s==other.s)
                    return true;

                return false;
            }

            public override int GetHashCode()
            {
                int hash = 17;
                hash = hash * 23 + i.GetHashCode();
                hash = hash * 23 + j.GetHashCode();
                hash = hash * 23 + s.GetHashCode();
                return hash;
            }
        }

        public class Model
        {
            public Cplex model { get; set; }

            public Instance Inst { get; set; }
            public Dictionary<X_index, double> TravelTimes { get; set; }

            public List<X_index> IndiciX { get; set; }
            public List<Z_index> IndiciZ { get; set; }
            public List<int> FeasLoc { get; set; }

            public List<int> FeasRS { get; set; }

            public Dictionary<int, int> ClosestRS2End { get; set; }
            public Dictionary<int, int> ClosestRS2Start { get; set; }

            public Dictionary<X_index, INumVar> Xij { get; set; }
            public Dictionary<Z_index, INumVar> Zijs { get; set; }
            public Dictionary<int, INumVar> Tj { get; set; }
            public Dictionary<int, INumVar> Fj { get; set; }

        }


       public static Model DroneMod = new Model();

        static void Main(string[] args)
        {
            if (args.Length < 2)
            {
                Console.Write("Usage: DroneRouter <instanceFileName> <TimeLimit>");
                return;
            }

            string instanceFileName = args[0];

            DroneRouter DR = new DroneRouter();

            Instance Inst = new Instance();

            DR.InstanceReader(instanceFileName, Inst);

            DroneMod.model = new Cplex();

            DroneMod.Inst = Inst;
            DroneMod.TravelTimes = new Dictionary<X_index, double>();
            DroneMod.IndiciX = new List<X_index>();
            DroneMod.IndiciZ = new List<Z_index>();
            DroneMod.ClosestRS2End = new Dictionary<int, int>();
            DroneMod.ClosestRS2Start = new Dictionary<int, int>();

            DR.ComputeDistances(Inst);

            List<Arc> ArcSet = DR.FeasibleArcs(Inst); // Lista archi ammissibili

            DR.ComputeZindex(Inst);

            DroneMod.Xij = new Dictionary<X_index, INumVar>();
            DroneMod.Zijs = new Dictionary<Z_index, INumVar>();
            DroneMod.Tj = new Dictionary<int, INumVar>();
            DroneMod.Fj = new Dictionary<int, INumVar>();

            DR.CreateVars(Inst);
            DR.CreateConstraints(Inst);
            DR.CreateObjective(Inst);

            DroneMod.model.ExportModel("DroneModel.lp");

            // OUTPUT CPLEX SOPPRESSO
            //DroneMod.model.SetOut(null);
            //DroneMod.model.SetWarning(null);

            double cpxTime = DroneMod.model.GetCplexTime();

            if (DroneMod.model.Solve())
            {
                Console.WriteLine("Risolto: " + DroneMod.model.GetObjective + " tempo = " + (DroneMod.model.GetCplexTime() - cpxTime) + " gap = " + DroneMod.model.GetMIPRelativeGap());
                DR.WriteSolution(Inst);
            }
            else 
            {
                Console.WriteLine("Non Risolto: " + DroneMod.model.GetStatus());
            }

            return;
        }

        public void CreateVars(Instance inst)
        {
            INumVar variab;

            foreach (var xv in DroneMod.IndiciX)
            {
                variab = DroneMod.model.BoolVar("X_" + xv.i + "_" + xv.j);
                DroneMod.Xij.Add(xv, variab);
            }

            foreach (var zv in DroneMod.IndiciZ)
            {
                variab = DroneMod.model.BoolVar("Z_" + zv.i + "_" + zv.j + "_" + zv.s);
                DroneMod.Zijs.Add(zv, variab);
            }

            foreach (var loc in DroneMod.FeasLoc)
            {
                variab = DroneMod.model.NumVar(0.0, inst.Tbig, NumVarType.Float, "T_" + loc);
                DroneMod.Tj.Add(loc, variab);
                variab = DroneMod.model.NumVar(0.0, 1.0, NumVarType.Float, "F_" + loc);
                DroneMod.Fj.Add(loc, variab);
            }

        }

        public void CreateConstraints(Instance inst)
        {
            ILinearNumExpr constr;
            constr = DroneMod.model.LinearNumExpr();

            //(11)
            foreach (var loc in DroneMod.FeasLoc)
            {
                constr = DroneMod.model.LinearNumExpr();
                foreach (var xv in DroneMod.IndiciX.Where(e => e.i == loc))
                {
                    constr.AddTerm(1.0, DroneMod.Xij[xv]);
                }
                DroneMod.model.AddEq(constr, 1.0, "OutDeg_" + loc);
            }

            //(12)
            foreach (var loc in DroneMod.FeasLoc)
            {
                constr = DroneMod.model.LinearNumExpr();
                foreach (var xv in DroneMod.IndiciX.Where(e => e.j == loc))
                {
                    constr.AddTerm(1.0, DroneMod.Xij[xv]);
                }
                DroneMod.model.AddEq(constr, 1.0, "InDeg_" + loc);
            }

            //(13)
            foreach (var loc in DroneMod.FeasLoc)
            {
                List<int> Rplus = DroneMod.IndiciZ.Where(e=> e.i==loc).Select(e=> e.s).Distinct().ToList();
                foreach (var rs in Rplus)
                {
                    constr = DroneMod.model.LinearNumExpr();
                    constr.AddTerm(1.0, DroneMod.Xij[new X_index(loc, rs)]);
                    foreach (var zind in DroneMod.IndiciZ.Where(e=> e.i==loc && e.s==rs))
                    {
                        constr.AddTerm(-1.0, DroneMod.Zijs[zind]);
                    }
                    DroneMod.model.AddGe(constr, 0.0, "StartAfterStop_" + loc + "_" + rs);
                }
            }

            //(14)
            foreach (var loc in DroneMod.FeasLoc)
            {
                List<int> Rminus = DroneMod.IndiciZ.Where(e => e.j == loc).Select(e => e.s).Distinct().ToList();
                foreach (var rs in Rminus)
                {
                    constr = DroneMod.model.LinearNumExpr();
                    constr.AddTerm(1.0, DroneMod.Xij[new X_index(rs, loc)]);
                    foreach (var zind in DroneMod.IndiciZ.Where(e => e.j == loc && e.s == rs))
                    {
                        constr.AddTerm(-1.0, DroneMod.Zijs[zind]);
                    }
                    DroneMod.model.AddGe(constr, 0.0, "FinishBeforeStop_" + rs + "_" + loc);
                }
            }

            //(15) - non include (16) se FeasRS non include nodo zero 
            foreach (var rs in DroneMod.FeasRS)
            {
                constr = DroneMod.model.LinearNumExpr();

                foreach (var xind in DroneMod.IndiciX.Where(e=> e.i==rs))
                {
                    constr.AddTerm(1.0, DroneMod.Xij[xind]);
                }

                foreach (var zind in DroneMod.IndiciZ.Where(e => e.s == rs))
                {
                    constr.AddTerm(-1.0, DroneMod.Zijs[zind]);
                }
                int initDrones = DroneMod.Inst.RSList.Where(e => e.IdRS == rs).Select(e => e.InitNumDrones).Single();
                DroneMod.model.AddLe(constr, initDrones , "ArrivalDepRS_" + rs);
            }

            //(16)  
            constr = DroneMod.model.LinearNumExpr();

            foreach (var xind in DroneMod.IndiciX.Where(e => e.i == 0))
            {
                constr.AddTerm(1.0, DroneMod.Xij[xind]);
            }

            int initDronesAS = DroneMod.Inst.RSList.Where(e => e.IdRS == 0).Select(e => e.InitNumDrones).Single();
            DroneMod.model.AddLe(constr, initDronesAS, "MaxDepFromAS");
            
            //(17)
            List<int> InitialRS = DroneMod.Inst.RSList.Where(e=> e.InitNumDrones>0).Select(e=>e.IdRS).ToList();
            foreach (var rs in InitialRS)
            {
                foreach (var xind in DroneMod.IndiciX.Where(e=> e.i==rs))
                {
                    constr = DroneMod.model.LinearNumExpr();
                    constr.AddTerm(1.0, DroneMod.Tj[xind.j]);
                    constr.AddTerm(-DroneMod.Inst.Tbig, DroneMod.Xij[xind]);

                    DroneMod.model.AddGe(constr, DroneMod.TravelTimes[xind] - DroneMod.Inst.Tbig, "ArrivalAfterStart_" + rs + "_" + xind.j);
                }
            }

            //(18)
            foreach (var xind in DroneMod.IndiciX)
            {
                if (!DroneMod.FeasLoc.Contains(xind.i) || !DroneMod.FeasLoc.Contains(xind.j))
                    continue;
                constr = DroneMod.model.LinearNumExpr();
                constr.AddTerm(1.0, DroneMod.Tj[xind.j]);
                constr.AddTerm(-1.0, DroneMod.Tj[xind.i]);
                constr.AddTerm(-DroneMod.Inst.Tbig, DroneMod.Xij[xind]);

                double servTime = DroneMod.Inst.LocList.Where(e => e.IdLoc == xind.i).Select(e => e.ServiceTime).Single();

                DroneMod.model.AddGe(constr, DroneMod.TravelTimes[xind] + servTime - DroneMod.Inst.Tbig, "ArrivalAfterLoc_" + xind.i + "_" + xind.j);
            }

            //(19)
            foreach (var zind in DroneMod.IndiciZ)
            {
                constr = DroneMod.model.LinearNumExpr();
                constr.AddTerm(1.0, DroneMod.Tj[zind.j]);
                constr.AddTerm(-1.0, DroneMod.Tj[zind.i]);
                constr.AddTerm(-DroneMod.Inst.Tbig, DroneMod.Zijs[zind]);

                double servTime = DroneMod.Inst.LocList.Where(e => e.IdLoc == zind.i).Select(e => e.ServiceTime).Single();

                DroneMod.model.AddGe(constr, DroneMod.TravelTimes[new X_index(zind.i, zind.s)] + servTime + DroneMod.Inst.Rho_TimeRecharge - DroneMod.Inst.Tbig, "ArrivalAfterRS_" + zind.i + "_" + zind.j + "_" + zind.s);
            }

            //(20)
            foreach (var xind in DroneMod.IndiciX)
            {
                if (!DroneMod.FeasLoc.Contains(xind.i) || !DroneMod.FeasLoc.Contains(xind.j))
                    continue;
                constr = DroneMod.model.LinearNumExpr();
                constr.AddTerm(1.0, DroneMod.Fj[xind.j]);
                constr.AddTerm(-1.0, DroneMod.Fj[xind.i]);
                constr.AddTerm(1.0, DroneMod.Xij[xind]);

                double servTime = DroneMod.Inst.LocList.Where(e => e.IdLoc == xind.i).Select(e => e.ServiceTime).Single();

                DroneMod.model.AddLe(constr, 1.0 - (DroneMod.Inst.Phi_BattConTravel*DroneMod.TravelTimes[xind] + DroneMod.Inst.Omega_BattConHover*servTime)/ DroneMod.Inst.MaxBattery, "SOCAfterLoc_" + xind.i + "_" + xind.j);
            }

            //(21)
            foreach (var loc in DroneMod.FeasLoc)
            {
                List<int> Rplus = DroneMod.IndiciZ.Where(e => e.i == loc).Select(e => e.s).Distinct().ToList();
                if(!Rplus.Contains(DroneMod.ClosestRS2End[loc]))
                    Rplus.Add(DroneMod.ClosestRS2End[loc]);

                foreach (var rs in Rplus)
                {
                    constr = DroneMod.model.LinearNumExpr();
                    constr.AddTerm(1.0, DroneMod.Fj[loc]);
                    constr.AddTerm(-1.0, DroneMod.Xij[new X_index(loc, rs)]);
                    double servTime = DroneMod.Inst.LocList.Where(e => e.IdLoc == loc).Select(e => e.ServiceTime).Single();

                    DroneMod.model.AddGe(constr, - 1.0 + (DroneMod.Inst.Phi_BattConTravel * DroneMod.TravelTimes[new X_index(loc, rs)] + DroneMod.Inst.Omega_BattConHover * servTime) / DroneMod.Inst.MaxBattery, "SOC2ReachRS_" + loc + "_" + rs);
                }
            }

            //(22)
            foreach (var loc in DroneMod.FeasLoc)
            {
                List<int> Rminus = DroneMod.IndiciZ.Where(e => e.j == loc).Select(e => e.s).Distinct().ToList();
                Rminus.Add(0); 
                if (!Rminus.Contains(DroneMod.ClosestRS2Start[loc]))
                    Rminus.Add(DroneMod.ClosestRS2Start[loc]);

                foreach (var rs in Rminus)
                {
                    constr = DroneMod.model.LinearNumExpr();
                    constr.AddTerm(1.0, DroneMod.Fj[loc]);
                    constr.AddTerm(1.0, DroneMod.Xij[new X_index(loc, rs)]);
                    double servTime = DroneMod.Inst.LocList.Where(e => e.IdLoc == loc).Select(e => e.ServiceTime).Single();

                    DroneMod.model.AddLe(constr, 2.0 + (DroneMod.Inst.Phi_BattConTravel * DroneMod.TravelTimes[new X_index(rs,loc)] + DroneMod.Inst.Omega_BattConHover * servTime) / DroneMod.Inst.MaxBattery, "SOCLocAfterRecharge_" + rs + "_" + loc);
                }
            }

        }

        public void CreateObjective(Instance inst)
        {
            // obiettivo
            ILinearNumExpr objectiveFun = DroneMod.model.LinearNumExpr();

            foreach (var xind in DroneMod.IndiciX)
            {
                objectiveFun.AddTerm(DroneMod.TravelTimes[xind], DroneMod.Xij[xind]);
            }
            DroneMod.model.AddObjective(ObjectiveSense.Minimize, objectiveFun, "ObjectiveDef");

        }
          
        public void WriteSolution(Instance inst)
        {
            StreamWriter fout = new StreamWriter("output_"+inst.Id+".csv", false);
            fout.WriteLine("RECORD FORMAT");

            // TO DO
            
            fout.Close();

        }

        public void InstanceReader(string fileName, Instance inst)
        {

            inst.LocList = new List<Location>();
            inst.RSList = new List<RechargeStopLoc>();

        }
      
        public void ComputeDistances(Instance inst)
        {
            DroneMod.TravelTimes = null;
        }

        public List<Arc> FeasibleArcs(Instance inst)
        {
            List<Arc> arcs = new List<Arc>();

            DroneMod.IndiciX = null;
            DroneMod.FeasLoc = null;
            DroneMod.FeasRS = null;
            DroneMod.ClosestRS2End = null;
            DroneMod.ClosestRS2Start = null;

            return arcs;

        }

        public void ComputeZindex(Instance inst)
        {
            
            DroneMod.IndiciZ = null;

        }

    }
}

