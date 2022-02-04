# Drones
### Authors: Massimo Paolucci, Antonio Sgorbissa, Carmine Recchiuto

Se generate delle istanze potreste implementare anche il metodo che le legge da file (è meglio salvare le istanze sperimentali in files testo, di solito, per replicare le campagne di test sperimentali). 
Nel progetto ho previsto questo metodo

 public void InstanceReader(string fileName, Instance inst)

che si aspetta in input il path del file con l'istanze e riempie un oggetto della classe Instance.
La classe Instance è definita qui sotto insieme alle classi Location (le locazioni da servire) e RechargeStopLoc (i punti in cui i droni possono ricaricare e sostare dove il punto con IdRS=0 corrisponde (se esiste) al punto di partenza di nuovi droni portati nell'area.

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
