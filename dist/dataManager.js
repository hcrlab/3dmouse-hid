let instance =null;

class DataManager{
    constructor(){
        if(!instance){
            this.data=[]
            instance=this;
        }
        return instance;

    }
    pushData(newData){
        this.data.push(newData);
    }
    getData(){
        return this.data;
    }
}
export default new DataManager();