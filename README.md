
## Mathematical Optimization Project
# The pallet-loading vehicle routing problem with stability constraints
M. T. Alonso, A. Martinez-Sykora, R. Alvarez-Valdes, F. Parreño

## Obiettivo
Instradare una flotta di k camion per soddisfare la richiesta di pallet da parte di n clienti mantenendo il costo più basso possibile. Tutto questo soddisfando i vincoli sul caricamento dei pallet e vincoli sull'instadamento, in maniera tale consegnare tutti i pallet richiesti e di avere su ogni camion un carico stabile posizionato in maniera tale da permettere lo scaricamento dei pallet in maniera semplice

## Sviluppo
Lo sviluppo è stato fatto in python utilizzando guroby con la licenza accademica.
E' stata sviluppata una classe per la risoluzione in cui oltre al metodo che modella il problema sono presenti anche altri metodi che creano una schermata per la visualizzazione del risultato.

![Tool visualizzazione](https://drive.google.com/uc?export=view&id=1HqZDp6JVdgL_mfTrLwwnS13vd5PgQWLN)

## Aggiunte
Oltre al modello "base" del paper si è voluta aggiungere la gestione pallet con altezze differenti e di camion con lunghezza e altezza diverse per provare ad estendere maggiormente l'utilizzabilità del modello.
La larghezza del camion rimane invariata in maniera tale da permettere due file di pallet su ogni camion

## Utilizzo
Nella classe chiamante:
`from TruckLoadingClass import solve_pallet_loading_vehicle_routing as ProblemSolver`(versione base)
or
`from TruckLoadingClassModified import solve_pallet_loading_vehicle_routing as ProblemSolver`(versione con aggiunte)

    solver = ProblemSolver("path_file_csv")  
    
    #attr: 1 -> visualizzazione grafica
    #attr: 0 -> no visualizzazione grafica
    solver.solve_problem(1)
## CSV dati
#truck + numero truck
dati truck: 		 nTruck|lunghezza|larghezza|altezza|pesoMax|pesoMaxAsseAnt|distanzaAsseAnt|pesoMaxAssePost|distanzaAssePost|
#pallet + numero pallet
dati pallet:
nPallet|lunghezza|larghezza|tipologiaPallet
#distance + nodes x nodes
matrice delle distanze
esempi:
<table>
	<tr>
		<th>Standard</th>
		<th> Aggiunte</th>
	</tr>
	<tr>		
		<td> 
		#truck 1  <br>
		1 13500 2450 2000 20411 10432 914 9979 12716  <br>
		#pallet 1  <br>
		1 1200 1600 1800  <br>
		#demand 16  <br>
		1 1 1000 1  <br>
		2 2 1000 1  <br>
		3 3 1000 1  <br>
		4 4 1000 1  <br>
		5 5 1000 1  <br>
		6 1 1000 1  <br>
		7 2 1000 1  <br>
		8 3 1000 1  <br>
		9 4 1000 1  <br>
		10 5 1000 1  <br>
		11 1 1000 1  <br>
		12 2 1000 1  <br>
		13 3 1000 1  <br>
		14 4 1000 1  <br>
		15 5 1000 1  <br>
		16 5 1000 1  <br>
		#distances 5 5  <br>
		0 1311 918 987 1240 1531  <br>
		1311 0 575 361 1127 1508  <br>
		918 575 0 243 644 1040  <br>
		987 361 243 0 873 1268  <br>
		1240 1127 644 873 0 397  <br>
		1531 1508 1040 1268 397 0
		</td>
		<td>
		#truck 2  <br>
		1 12000 2450 2800 20411 10432 914 9979 10700  <br>
		2 10000 2400 2200 15000 8700 700 6300 8800 <br> 
		#pallet 3  <br>
		1 1200 1600 1800  <br>
		2 1200 1600 1500  <br>
		3 1200 1600 2400  <br>
		#demand 16  <br>
		1 1 1345 1  <br>
		2 2 745 1  <br>
		3 3 891 2  <br>
		4 4 1335 2  <br>
		5 5 1124 2  <br>
		6 1 515 1  <br>
		7 2 1440 1  <br>
		8 3 1238 3  <br>
		9 4 818 1  <br>
		10 5 536 2  <br>
		11 1 1000 2  <br>
		12 2 659 1  <br>
		13 3 1187 1  <br>
		14 4 896 1  <br>
		15 5 880 1  <br>
		16 5 774 3  <br>
		#distances 5 5  <br>
		0 1311 918 987 1240 1531  <br>
		1311 0 575 361 1127 1508  <br>
		918 575 0 243 644 1040  <br>
		987 361 243 0 873 1268  <br>
		1240 1127 644 873 0 397  <br>
		1531 1508 1040 1268 397 0<br>
		</td>
	</tr>
</table>
