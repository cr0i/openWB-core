<template>
	<div class="d-flex flex-column p-3">
		<div class="subtitle mb-4">Zeitpläne:</div>
		<div v-if="plans.length == 0" class="warning p-5">
			Es sind noch keine Pläne definiert. Zeitpläne können in der Web-App
			festgelegt werden.
		</div>

		<table class="table table-dark">
			<thead>
				<tr>
					<th></th>
					<th>Von</th>
					<th>Bis</th>
					<th>Ladestrom</th>
					<th>Wiederholung</th>
				</tr>
			</thead>
			<tbody>
				<tr v-for="(plan, i) in plans" :key="i" :style="cellStyle(i)">
					<td>
						<SwitchInput
							v-model="plan.active"
							@update:model-value="updatePlanState(i)"
						>
						</SwitchInput>
					</td>
					<td>{{ plan.time[0] }}</td>
					<td>{{ plan.time[1] }}</td>
					<td>{{ plan.current }}A</td>
					<td>{{ freqNames[plan.frequency.selected] }}</td>
				</tr>
			</tbody>
		</table>
	</div>
</template>

<script setup lang="ts">
import { computed } from 'vue'
import { ChargePoint } from '../model'
import { updateChargeTemplate } from '@/assets/js/sendMessages'
import SwitchInput from '@/components/shared/SwitchInput.vue'

const freqNames: { [key: string]: string } = {
	daily: 'Täglich',
	once: 'Einmal',
	weekly: 'Wöchentlich',
}
const props = defineProps<{
	chargePoint: ChargePoint
}>()
const plans = computed(() => {
	return props.chargePoint.chargeTemplate?.time_charging.plans ?? []
})
function updatePlanState(i: number) {
	props.chargePoint.chargeTemplate!.chargemode.scheduled_charging.plans[
		i
	]!.active = plans.value[i].active
	updateChargeTemplate(props.chargePoint.id)
}
function cellStyle(key: number) {
	const style = plans.value[key].active ? 'bold' : 'regular'
	return { 'font-weight': style }
}
</script>

<style scoped>
.timeplantable {
	justify-content: center;
	gap: 20px;
}
.subtitle {
	font-size: var(--font-large);
	font-weight: bold;
}
.warning {
	font-size: var(--font-large);
	font-weight: bold;
	color: var(--color-evu);
}
td {
	background-color: var(--color-bg) !important;
}
th {
	background-color: var(--color-bg) !important;
}
</style>
