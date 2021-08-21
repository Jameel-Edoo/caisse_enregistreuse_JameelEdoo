#include <iostream>
#include <iomanip>

using namespace std;

double random(double max);
double calcule_monnaie(double cost, double payment);


int main()
{

    int _2k;
    int _1k;
    int _5h;
    int _2h;
    int _1h;
    int fifty;
    int twenty_five;
    int twenty;
    int ten;
    int five;
    int one;
    int _50c;
    int _20c;
    int _5c;


    int A_2k = 5;
    int A_1k = 5;
    int A_5h = 10;
    int A_2h = 10;
    int A_1h = 10;
    int A_fifty = 20;
    int A_twenty_five = 20;
    int A_twenty = 20;
    int A_ten = 20;
    int A_five = 20;
    int A_one = 50;
    int A_50c = 50;
    int A_20c = 50;
    int A_5c = 50;


    double change = 0;
    double total_Amount = (A_2k*2000 + A_1k*1000 + A_5h*500 + A_2h*200 + A_1h*100 + A_fifty*50 + A_twenty_five*25 + A_twenty*20 + A_ten*10 + A_five*5 + A_one + A_50c*0.5 + A_20c*0.2 + A_5c*0.05);


    do {

        double cost = random(10000);
        double payment;

        cout << "Le prix a payer est: " << cost << endl;

        cout << "Veuillez entrez la valeur que vous allez donner pour payer: " << endl;
        cin >> payment;

        while(payment < cost) {
        cout << "Donnez une valeur superieur au prix a payer svp: " << endl;
        cin >> payment;
        }


        double change = calcule_monnaie(cost, payment);
        total_Amount = (total_Amount - change);



        cout << "Le montant totale en echange est: ";
        cout << fixed << setprecision(2) << change << endl;
    
        int total = int(100)*change;



        _2k = total/200000;
        total = total % 200000;
    
        _1k = total/100000;
        total = total % 100000;
   
    	_5h = total/50000;
    	total = total % 50000;
   
   	_2h = total/20000;
  	total = total % 20000;
   
  	_1h = total/10000;
   	total = total % 10000;
   
   	fifty = total/5000;
   	total = total % 5000;
   
   	twenty_five = total/2500;
   	total = total % 2500;
   
   	twenty = total/2000;
   	total = total % 2000;
   
   	ten = total/1000;
   	total = total % 1000;
   
    	five = total/500;
    	total = total % 500;
   
    	one = total/100;
    	total = total % 100;
   
    	_50c = total/50;
    	total = total % 50;
   
    	_20c = total/20;
    	total = total % 20;
   
    	_5c = total/5;
   

	    if ((_2k <= A_2k) and (_1k <= A_1k) and (_5h <= A_5h) and (_2h <= A_2h) and (_1h <= A_1h) and (fifty <= A_fifty) and (twenty_five <= A_twenty_five) and (twenty <= A_twenty) and (ten <= A_ten) and (five <= A_five) and (one <= A_one) and (_50c <= A_50c) and (_20c <= A_20c) and (_5c <= A_5c)) {

		 

    	        cout << "Voici votre monnaie d'echange en billets et pieces: " << endl;

    	        if (_2k != 0) {
        	cout << "(Billets 2000): " << _2k << endl;
		A_2k = (A_2k - _2k);
    	        }

    	        if (_1k != 0) {
        	cout << "(Billets 1000): " << _1k << endl;
		A_1k = (A_1k - _1k);
    	        }

    	        if (_5h != 0) {
        	cout << "(Billets 500): " << _5h << endl;
		A_5h = (A_5h - _5h);
    	        }

    	        if (_2h != 0) {
        	cout << "(Billets 200): " << _2h << endl;
		A_2h = (A_2h - _2h);
        	    }

    	        if (_1h != 0) {
        	cout << "(Billets 100): " << _1h << endl;
		A_1h = (A_1h - _1h);
    	        }
    
    	        if (fifty != 0) {
        	cout << "(Billets 50): " << fifty << endl;
		A_fifty = (A_fifty - fifty);
    	        }

    	        if (twenty_five != 0) {
        	cout << "(Billets 25): " << twenty_five << endl;
		A_twenty_five = (A_twenty_five - twenty_five);
   	        }

        	if (twenty != 0) {
        	cout << "(Pieces 20): " << twenty << endl;
		A_twenty = (A_twenty - twenty);
        	}

        	if (ten != 0) {
        	cout << "(Pieces 10): " << ten << endl;
		A_ten = (A_ten - ten);
        	}

        	if (five != 0) {
        	cout << "(Pieces 5): " << five << endl;
		A_five = (A_five - five);
        	}

        	if (one != 0) {
        	cout << "(Pieces 1): " << one << endl;
		A_one = (A_one - one);
    	        }

    	        if (_50c != 0) {
            	cout << "(Pieces 50 sous): " << _50c << endl;
		A_50c = (A_50c - _50c);
        	}

        	if (_20c != 0) {
        	cout << "(Pieces 20 sous): " << _20c << endl;
		A_20c = (A_20c - _20c);
        	}

        	if (_5c != 0) {
        	cout << "(Pieces 5 sous): " << _5c << endl;
		A_5c = (A_5c - _5c);
        	}
	    }

	else {
	    cout << "Il n'y a pas asser de billet et pieces dans la caisse pour l'echange, desole." << endl;
    	    break;
        }
    }
    while (change < total_Amount);

    cout << "Le montant totale en echange est trop grande pour notre caisse, desole." << endl;



    return 0;

}



double random(double max)
{
    srand((unsigned) time(0));
    return (max / RAND_MAX) * rand();
}


double calcule_monnaie(double cost, double payment)
{
    return (payment - cost);
}






















