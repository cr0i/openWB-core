import{_ as r,p as n,k as p,l,A as o,L as s,u as a,q as d,x as u}from"./vendor-6e5b23b4.js";import"./vendor-sortablejs-b3476726.js";const _={name:"DeviceKostalPlenticoreBat",emits:["update:configuration"],props:{configuration:{type:Object,required:!0},deviceId:{default:void 0},componentId:{required:!0}},methods:{updateConfiguration(e,t=void 0){this.$emit("update:configuration",{value:e,object:t})}}},f={class:"device-kostalplenticore-bat"},m={class:"small"};function b(e,t,g,h,v,k){const i=n("openwb-base-heading"),c=n("openwb-base-alert");return p(),l("div",f,[o(i,null,{default:s(()=>[a(" Einstellungen für Kostal Plenticore Batteriespeicher "),d("span",m,"(Modul: "+u(e.$options.name)+")",1)]),_:1}),o(c,{subtype:"info"},{default:s(()=>[a(" Diese Komponente benötigt keine Einstellungen. ")]),_:1})])}const $=r(_,[["render",b],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/kostal_plenticore/bat.vue"]]);export{$ as default};