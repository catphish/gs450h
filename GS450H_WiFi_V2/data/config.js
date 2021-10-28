setInterval(function ( ) {
  var xmlhttp = new XMLHttpRequest();
  var url = "/data.json";

  xmlhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var response_data = JSON.parse(this.responseText);
      var table = document.getElementById("data-table");
      response_data.forEach(function (item, index) {
        var row = document.getElementById("row-" + item[0]);
        if(row == null) {
          row = document.createElement("TR");
          row.id = "row-" + item[0];
          table.appendChild(row);

          var name_td = document.createElement("TD");
          name_td.id = "name-" + item[0];
          row.appendChild(name_td);

          var value_td = document.createElement("TD");
          value_td.id = "value-" + item[0];
          row.appendChild(value_td);
          if(item[2] == 1) {
            var input_td = document.createElement("TD");
            row.appendChild(input_td);

            var button_td = document.createElement("TD");
            row.appendChild(button_td);

            var input_field = document.createElement("INPUT");
            input_field.id = "input-" + item[0];
            input_td.appendChild(input_field);

            var button = document.createElement("BUTTON");
            button.id = "button-" + item[0];
            button.innerHTML = "Set"
            button.onclick = function() {
              var value = document.getElementById("input-" + item[0]).value;
              command = String(item[0]).padStart(2, '0') + ',' + value;
              console.log(command);
              sendCommand(command);
            };
            button_td.appendChild(button);
          }
        }
        name_td = document.getElementById("name-" + item[0]);
        name_td.innerHTML = item[1];
        value_td = document.getElementById("value-" + item[0]);
        value_td.innerHTML = item[3];
      });
    }
  };
  xmlhttp.open("GET", url, true);
  xmlhttp.send();
}, 500 ) ;

function sendCommand(command) {
  var xmlhttp = new XMLHttpRequest();
  xmlhttp.open("POST", "/command", true);
  xmlhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  xmlhttp.send("command="+command);
}
