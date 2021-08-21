#include <iostream>

using namespace std;

double random(double max);

int main()
{
    double cost = random(10000);
    double payment;

    cout << "Le prix a payer est: " << cost << endl;

    cout << "Veuillez entrez la valeur que vous allez donner pour payer: " << endl;
    cin >> payment;

    while(payment < cost) {
    cout << "Donnez une valeur superieur au prix a payer svp: " << endl;
    cin >> payment;
    }

    cout << cost << payment;

}



double random(double max)
{
    srand((unsigned) time(0));
    return (max / RAND_MAX) * rand();
}
