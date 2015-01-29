#include <iostream>
#include <ctime>
using namespace std;

int print(char* m){
        cout<<m<<endl;
        return 0;
}


int wait(int (*foo)(char*),char* m, int sec) {
        time_t timer;
        timer = time(NULL);
        for(int i=0;i<sec;i++)
        {
                while(time(NULL)<=timer+i){}
                cout<<time(NULL)<<endl;
                
        }
        int error = (*foo)(m);
        return error;
}

int main() {
        char m[]="This is printed after I kept you waiting long enough.";
        int error=wait(&print,m,6);
        cout<<"You have loco motives\n";
        cout<<"Error Code: "<<error<<endl;
        return 0;
}