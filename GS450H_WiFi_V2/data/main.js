var voltage;
var mg1RPM;
var mg2RPM;
var waterTemp;

setInterval(function ( ) {
  var xmlhttp = new XMLHttpRequest();
  xmlhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var response_data = JSON.parse(this.responseText);
      point = voltage.series[0].points[0];
      point.update(response_data["voltage"]);
      point = waterTemp.series[0].points[0];
      point.update(response_data["water_temp"]);
      point = mg1RPM.series[0].points[0];
      point.update(response_data["mg1_speed"]);
      point = mg2RPM.series[0].points[0];
      point.update(response_data["mg2_speed"]);
    }
  };
  xmlhttp.open("GET", "/data.json", true);
  xmlhttp.send();
}, 500 ) ;

var gaugeOptions = {
    chart: {
        type: 'solidgauge',
		backgroundColor: 'rgba(0,0,0,0)'
    },
    title: null,
    pane: {
        center: ['50%', '85%'],
        size: '140%',
        startAngle: -90,
        endAngle: 90,
        background: {
            backgroundColor:
                Highcharts.defaultOptions.legend.backgroundColor || '#EEE',
            innerRadius: '60%',
            outerRadius: '100%',
            shape: 'arc'
        }
    },
    exporting: {
        enabled: false
    },
    tooltip: {
        enabled: false
    },
    // the value axis
    yAxis: {
        stops: [
            [0.1, '#55BF3B'], // green
            [0.5, '#DDDF0D'], // yellow
            [0.9, '#DF5353']  // red
        ],
        lineWidth: 0,
        tickWidth: 0,
        minorTickInterval: null,
        tickAmount: 2,
        title: {
            y: -70
        },
        labels: {
            y: 16
        }
    },
    plotOptions: {
        solidgauge: {
            dataLabels: {
                y: 5,
                borderWidth: 0,
                useHTML: true
            }
        }
    }
};

document.addEventListener("DOMContentLoaded", function(event) {
    //Pack Voltage
    voltage = Highcharts.chart('container-voltage', Highcharts.merge(gaugeOptions, {
        yAxis: {
            min: 0,
            max: 700,
            title: {
                text: 'Voltage'
            }
        },
        credits: {
            enabled: false
        },
        series: [{
            name: 'Pack Voltage',
            data: [0],
            dataLabels: {
                format:
                    '<div style="text-align:center">' +
                    '<span style="font-size:25px">{y}</span><br/>' +
                    '<span style="font-size:12px;opacity:0.4">Volts</span>' +
                    '</div>'
            },
            tooltip: {
                valueSuffix: ' Volts'
            }
        }]
    }));

    //MG1 RPM
    mg1RPM = Highcharts.chart('container-mg1RPM', Highcharts.merge(gaugeOptions, {
        yAxis: {
            min: 0,
            max: 10000,
            title: {
                text: 'MG1 RPM'
            }
        },
        credits: {
            enabled: false
        },
        series: [{
            name: 'MG1 RPM',
            data: [0],
            dataLabels: {
                format:
                    '<div style="text-align:center">' +
                    '<span style="font-size:25px">{y}</span><br/>' +
                    '<span style="font-size:12px;opacity:0.4">RPM</span>' +
                    '</div>'
            },
            tooltip: {
                valueSuffix: ' RPM'
            }
        }]
    }));

    //MG2 RPM
    var mg2RPM = Highcharts.chart('container-mg2RPM', Highcharts.merge(gaugeOptions, {
        yAxis: {
            min: 0,
            max: 10000,
            title: {
                text: 'MG2 RPM'
            }
        },
        credits: {
            enabled: false
        },
        series: [{
            name: 'MG2 RPM',
            data: [0],
            dataLabels: {
                format:
                    '<div style="text-align:center">' +
                    '<span style="font-size:25px">{y}</span><br/>' +
                    '<span style="font-size:12px;opacity:0.4">RPM</span>' +
                    '</div>'
            },
            tooltip: {
                valueSuffix: ' RPM'
            }
        }]
    }));

    //Water Temperature
    waterTemp = Highcharts.chart('container-waterTemp', Highcharts.merge(gaugeOptions, {
        yAxis: {
            min: -20,
            max: 120,
            title: {
                text: 'Coolant Temperature'
            }
        },
        credits: {
            enabled: false
        },
        series: [{
            name: 'Water Temperature',
            data: [0],
            dataLabels: {
                format:
                    '<div style="text-align:center">' +
                    '<span style="font-size:25px">{y}</span><br/>' +
                    '<span style="font-size:12px;opacity:0.4">&#8451</span>' +
                    '</div>'
            },
            tooltip: {
                valueSuffix: ' &#8451'
            }
        }]
    }));

});

