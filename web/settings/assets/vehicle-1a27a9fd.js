import{V as l}from"./VehicleConfig-c2018fd2.js";import{_ as a,u,k as p,l as d,G as t,E as r,y as s}from"./vendor-809787c9.js";import"./vendor-fortawesome-e760f6db.js";import"./index-f9dddb60.js";import"./vendor-bootstrap-5ce91dd7.js";import"./vendor-jquery-49acc558.js";import"./vendor-axios-57a82265.js";import"./vendor-sortablejs-d99a4022.js";import"./dynamic-import-helper-be004503.js";const m={name:"VehicleSocBmw",mixins:[l]},v={class:"vehicle-soc-bmw"};function f(o,e,g,w,b,V){const i=u("openwb-base-text-input");return p(),d("div",v,[t(i,{title:"Benutzername",required:"",subtype:"user","model-value":o.vehicle.configuration.user_id,"onUpdate:modelValue":e[0]||(e[0]=n=>o.updateConfiguration(n,"configuration.user_id"))},{help:r(()=>e[3]||(e[3]=[s(" Der Benutzername für die Anmeldung an den BMW-Servern. ")])),_:1},8,["model-value"]),t(i,{title:"Kennwort",required:"",subtype:"password","model-value":o.vehicle.configuration.password,"onUpdate:modelValue":e[1]||(e[1]=n=>o.updateConfiguration(n,"configuration.password"))},{help:r(()=>e[4]||(e[4]=[s(" Das Passwort für die Anmeldung an den BMW-Servern. ")])),_:1},8,["model-value"]),t(i,{title:"VIN",required:"","model-value":o.vehicle.configuration.vin,"onUpdate:modelValue":e[2]||(e[2]=n=>o.updateConfiguration(n,"configuration.vin"))},{help:r(()=>e[5]||(e[5]=[s(" Die Fahrgestellnummer des Fahrzeugs. ")])),_:1},8,["model-value"])])}const y=a(m,[["render",f],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/vehicles/bmw/vehicle.vue"]]);export{y as default};