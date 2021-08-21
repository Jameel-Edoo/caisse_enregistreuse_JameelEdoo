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
   

    cout << "Voici votre monnaie d'echange en billets et pieces: " << endl;

    if (_2k != 0) {
        cout << "(Billets 2000): " << _2k << endl;
    }

    if (_1k != 0) {
        cout << "(Billets 1000): " << _1k << endl;
    }

    if (_5h != 0) {
        cout << "(Billets 500): " << _5h << endl;
    }

    if (_2h != 0) {
        cout << "(Billets 200): " << _2h << endl;
    }

    if (_1h != 0) {
        cout << "(Billets 100): " << _1h << endl;
    }

    if (fifty != 0) {
        cout << "(Billets 50): " << fifty << endl;
    }

    if (twenty_five != 0) {
        cout << "(Billets 25): " << twenty_five << endl;
    }

    if (twenty != 0) {
        cout << "(Pieces 20): " << twenty << endl;
    }

    if (ten != 0) {
        cout << "(Pieces 10): " << ten << endl;
    }

    if (five != 0) {
        cout << "(Pieces 5): " << five << endl;
    }

    if (one != 0) {
        cout << "(Pieces 1): " << one << endl;
    }

    if (_50c != 0) {
        cout << "(Pieces 50 sous): " << _50c << endl;
    }

    if (_20c != 0) {
        cout << "(Pieces 20 sous): " << _20c << endl;
    }

    if (_5c != 0) {
        cout << "(Pieces 5 sous): " << _5c << endl;
    }






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






















