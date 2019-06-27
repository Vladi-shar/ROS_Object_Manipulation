

(function (doc){
    let  cell_click = (cell) => {

        if (cell.currentTarget.firstChild.style.display == "block"){
            cell.currentTarget.firstChild.style.display = "none";
           
        }else
       { cell.currentTarget.firstChild.style.display = "block";}

        if (doc.querySelector("#chkColor:checked"))
        {
            cell.currentTarget.firstChild.style.backgroundColor = "red";
            let col = cell.currentTarget.firstChild.getAttribute("col_id");
            let row = cell.currentTarget.firstChild.getAttribute("row_id");
             let other_cell =doc.querySelector(`#target_table [row_id = '${row}'][col_id = '${col}']`);
            other_cell.style.backgroundColor = "red";

            if ( other_cell.style.display=="block"){
                other_cell.style.display="none";
            }else{
                other_cell.style.display="block";
            }
        }
    };
    let  createElementFromHTML = (htmlString)=> {
        var div = document.createElement('div');
        div.className="table_cell";
        div.innerHTML = htmlString.trim();
      
        return div; 
      }

        let detachChildrenElement = (element)=>{
                var e = document.querySelector(element); 
                var child = e.lastElementChild;  
                while (child) { 
                    e.removeChild(child); 
                    child = e.lastElementChild; 
                } 
             
        }


    document.addEventListener("DOMContentLoaded", function(event) { 


        var button = doc.querySelector("#createBtn");
        var sendButton = doc.querySelector("#sendBtn");
        let create_click = ()=>{
            doc.querySelector(".switch").style.display="inline-block";
            doc.querySelector("#sendBtn").style.display="inline-block";
            detachChildrenElement("#target_table");
            detachChildrenElement("#init_table");
            var target_table = doc.querySelector("#target_table");
            var init_table = doc.querySelector("#init_table");

            var rows_number = doc.querySelector("#rows").value;
            var cols_number = doc.querySelector("#rows").value;

            doc.querySelector("#init_table").style.gridTemplateColumns =" 55px".repeat(cols_number);
            doc.querySelector("#target_table").style.gridTemplateColumns =" 55px".repeat(cols_number);
          
          
            for (let current_col=0; current_col<cols_number; current_col++){
                
                for (let current_row=0; current_row<rows_number; current_row++){

                    let newDivTarget = createElementFromHTML(
                        `<div row_id="${current_row}" col_id="${current_col}" class="innerCell"> </div> `);
                    let newInitCell = createElementFromHTML(
                    `<div row_id="${current_row}" col_id="${current_col}" class="innerCell"> </div> `);
                    newDivTarget.addEventListener("click", cell_click);
                    newInitCell.addEventListener("click", cell_click);
                    target_table.append(newDivTarget); 
                    init_table.append (newInitCell);
                }
       
            }
        };
        let sendMatrix = () => {
            console.log("send matrix");
            var matrices = createMatrix();
            $.post("http://localhost:3000/matrix",{matrices: JSON.stringify(matrices)}, function(data){
            console.log(data);
            if(data=='None\n'){
                $("#answer").html("Couldn't calculate a path...");
            }
            else{
                $("#answer").html("Output String:<br><br>" + data);
            }
            
          });
        }

        let createMatrix = () => {
            var rows_number = doc.querySelector("#rows").value;
            var cols_number = doc.querySelector("#rows").value;


            var init_matrix = [];
            var target_matrix = [];


            for (var i = 0; i < rows_number; i++){
                var init_tmp = [];
                var target_tmp = [];
                for (var j = 0; j < cols_number; j++){
                    if ($(`#target_table [row_id='${i}'][col_id='${j}']`).css('display') == 'block'){
                        target_tmp.push(1);
                    }
                    else{
                        target_tmp.push(0);
                    }
                    if ($(`#init_table [row_id='${i}'][col_id='${j}']`).css('display') == 'block'){
                        init_tmp.push(1);
                    }
                    else{
                        init_tmp.push(0);
                    }
                }
                init_matrix.push(init_tmp);
                target_matrix.push(target_tmp);
            }
            var return_matrix = [];
            return_matrix.push(init_matrix);
            return_matrix.push(target_matrix);
            return return_matrix;
        }


        button.addEventListener("click", create_click);
        sendButton.addEventListener("click", sendMatrix);
      })
    



})(document);
