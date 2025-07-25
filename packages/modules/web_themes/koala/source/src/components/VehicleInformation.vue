<template>
  <BaseCarousel
    v-if="vehicleIds.length <= cardViewBreakpoint"
    :items="vehicleIds"
    :card-width="cardWidth"
  >
    <template #item="{ item }">
      <VehicleCard :vehicle-id="item" @card-width="cardWidth = $event" />
    </template>
  </BaseCarousel>

  <BaseTable
    v-else
    :items="vehicleIds"
    :row-data="tableRowData"
    :column-config="isMobile ? columnConfigMobile : columnConfigDesktop"
    :search-input-visible="searchInputVisible"
    :table-height="isMobile ? '35vh' : '45vh'"
    v-model:filter="filter"
    :columns-to-search="['name', 'manufacturer', 'model']"
    :row-expandable="true"
    @row-click="onRowClick"
  >
    <!-- "col" = column must match Quasar naming convention -->
    <template #body-cell-plugged="slotProps">
      <q-td :class="`text-${slotProps.col.align}`">
        <ChargePointStateIcon :vehicle-id="slotProps.row.id" />
      </q-td>
    </template>
    <template #row-expand="slotProps">
      <VehicleConnectionStateIcon :vehicle-id="slotProps.row.id" />
    </template>
  </BaseTable>

  <!-- VehicleCard Dialog -->
  <q-dialog
    v-model="modalChargeVehicleCardVisible"
    transition-show="fade"
    transition-hide="fade"
    :backdrop-filter="$q.screen.width < 385 ? '' : 'blur(4px)'"
  >
    <div class="dialog-content">
      <VehicleCard
        v-if="selectedVehicleId !== null"
        :vehicle-id="selectedVehicleId"
      >
        <template #card-footer>
          <div class="card-footer">
            <q-btn
              color="primary"
              flat
              no-caps
              v-close-popup
              class="close-button"
              size="md"
              >Schließen</q-btn
            >
          </div>
        </template>
      </VehicleCard>
    </div>
  </q-dialog>
</template>

<script setup lang="ts">
import { computed, ref } from 'vue';
import { useMqttStore } from 'src/stores/mqtt-store';
import { Platform } from 'quasar';
import BaseCarousel from 'src/components/BaseCarousel.vue';
import BaseTable from 'src/components/BaseTable.vue';
import { VehicleRow } from 'src/components/models/table-model';
import ChargePointStateIcon from 'src/components/ChargePointStateIcon.vue';
import VehicleConnectionStateIcon from './VehicleConnectionStateIcon.vue';
import VehicleCard from 'src/components/VehicleCard.vue';
import { ColumnConfiguration } from 'src/components/models/table-model';

const cardWidth = ref<number | undefined>(undefined);

const mqttStore = useMqttStore();
const isMobile = computed(() => Platform.is.mobile);
const modalChargeVehicleCardVisible = ref(false);
const selectedVehicleId = ref<number | null>(null);
const filter = ref('');
const searchInputVisible = computed(
  () => mqttStore.themeConfiguration?.vehicle_table_search_input_field,
);
const cardViewBreakpoint = computed(
  () => mqttStore.themeConfiguration?.vehicle_card_view_breakpoint || 4,
);
const vehicles = computed(() => mqttStore.vehicleList);
const vehicleIds = computed(() => vehicles.value.map((vehicle) => vehicle.id));

const tableRowData = computed<(id: number) => VehicleRow>(() => {
  return (id: number) => {
    const vehicle = mqttStore.vehicleList.find((vehicle) => vehicle.id === id);
    const name = vehicle?.name || 'keine Angabe';
    const vehicleState = mqttStore.vehicleConnectionState(id);
    const plugState = vehicleState.some((vehicle) => vehicle.plugged);
    const chargeState = vehicleState.some((vehicle) => vehicle.charging);
    const info = mqttStore.vehicleInfo(id);
    const manufacturer = info?.manufacturer || 'keine Angabe';
    const model = info?.model || 'keine Angabe';
    const soc = mqttStore.vehicleSocValue(id);
    const vehicleSocValue = soc !== undefined ? `${Math.round(soc)}%` : '–';
    return {
      id,
      name,
      manufacturer,
      model,
      plugState,
      chargeState,
      vehicleSocValue,
    };
  };
});

const columnConfigDesktop: ColumnConfiguration[] = [
  { field: 'name', label: 'Fahrzeug' },
  { field: 'manufacturer', label: 'Hersteller' },
  { field: 'model', label: 'Modell' },
  { field: 'plugged', label: 'Status', align: 'center' },
  { field: 'vehicleSocValue', label: 'Ladestand', align: 'right' },
];

const columnConfigMobile: ColumnConfiguration[] = [
  { field: 'name', label: 'Fahrzeug' },
  { field: 'plugged', label: 'Status', align: 'center' },
  { field: 'vehicleSocValue', label: 'Ladestand', align: 'right' },
];

const onRowClick = (row: VehicleRow) => {
  selectedVehicleId.value = row.id;
  modalChargeVehicleCardVisible.value = true;
};
</script>

<style scoped>
.dialog-content {
  width: auto;
  max-width: 24em;
}

.close-button {
  position: absolute;
  bottom: 0.4em;
  right: 0.4em;
  z-index: 1;
  background: transparent;
}

.card-footer {
  height: 1.9em;
}
</style>
