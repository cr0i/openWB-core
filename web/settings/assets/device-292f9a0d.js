import{D as r}from"./HardwareInstallation-9c1cfe5a.js";import{_ as a,u as t,k as d,l,G as s,E as u,y as m}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./index-d7731c53.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const c={name:"DeviceYouless",mixins:[r]},_={class:"device-youless"};function f(o,e,v,b,g,x){const n=t("openwb-base-heading"),i=t("openwb-base-text-input");return d(),l("div",_,[s(n,null,{default:u(()=>e[1]||(e[1]=[m(" Einstellungen für Youless ")])),_:1}),s(i,{title:"IP oder Hostname",subtype:"host",required:"","model-value":o.device.configuration.ip_address,"onUpdate:modelValue":e[0]||(e[0]=p=>o.updateConfiguration(p,"configuration.ip_address"))},null,8,["model-value"])])}const h=a(c,[["render",f],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/youless/youless/device.vue"]]);export{h as default};