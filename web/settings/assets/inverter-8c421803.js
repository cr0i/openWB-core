import{_ as l,q as o,l as m,m as _,A as r,K as i,v as n,u as s,x as f}from"./vendor-b78ff8c0.js";import"./vendor-sortablejs-116030fd.js";const v={name:"DeviceDiscovergyInverter",emits:["update:configuration"],props:{configuration:{type:Object,required:!0},deviceId:{default:void 0},componentId:{required:!0}},methods:{updateConfiguration(t,e=void 0){this.$emit("update:configuration",{value:t,object:e})}}},g={class:"device-discovergy-inverter"},b={class:"small"},h=s("a",{href:"https://api.discovergy.com/public/v1/meters",target:"_blank",rel:"noopener noreferrer"}," https://api.discovergy.com/public/v1/meters ",-1);function w(t,e,a,y,x,d){const u=o("openwb-base-heading"),c=o("openwb-base-text-input");return m(),_("div",g,[r(u,null,{default:i(()=>[n(" Einstellungen für Discovergy Wechselrichter "),s("span",b,"(Modul: "+f(t.$options.name)+")",1)]),_:1}),r(c,{title:"Meter-ID",required:"","model-value":a.configuration.meter_id,"onUpdate:modelValue":e[0]||(e[0]=p=>d.updateConfiguration(p,"configuration.meter_id"))},{help:i(()=>[n(" Um die ID herauszufinden mit dem Browser die Adresse "),h,n(" aufrufen und dort Benutzername und Passwort eingeben. Hier wird nun u.a. die ID des Zählers angezeigt. ")]),_:1},8,["model-value"])])}const I=l(v,[["render",w],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/discovergy/inverter.vue"]]);export{I as default};