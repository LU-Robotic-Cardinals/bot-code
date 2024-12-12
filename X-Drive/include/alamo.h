struct Alamo {
public:
    int remembered = 0;

    Alamo(int num){
        remembered = num;
    }
    
    int remember_the_alamo(){
        return ++remembered;
        
    }
};