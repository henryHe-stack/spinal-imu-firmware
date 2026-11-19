/* S09 compile-only Cortex-M3 ABI contract, deliberately NOT MCU-linked.
 * Build using audit_target.ps1. Host sizeof is not evidence of target ABI.
 * Update these only after reviewing new target ELF/queue/stack budgets. */
#include "neck_app.h"
#include "neck_clock.h"
#if !defined(__arm__)
#error "Use ARM compiler for this target ABI check."
#endif
#define ABI(name,condition) typedef char neck_abi_##name[(condition)?1:-1]
ABI(pointer_32,sizeof(void *)==4);
ABI(double_64,sizeof(double)==8);
ABI(frame_152,sizeof(NeckFrame)==152);
ABI(orientation_176,sizeof(NeckOrientationFrame)==176);
ABI(acquisition_328,sizeof(NeckAcquisitionFrame)==328);
ABI(quality_output_56,sizeof(NeckQualityOutput)==56);
ABI(quality_context_256,sizeof(NeckQualityContext)==256);
ABI(pipeline_context_384,sizeof(NeckPipelineContext)==384);
ABI(posture_context_160,sizeof(NeckPostureContext)==160);
ABI(relative_context_304,sizeof(NeckRelativeContext)==304);
ABI(relative_output_648,sizeof(NeckRelativeOutput)==648);
ABI(processing_parameters_248,sizeof(NeckProcessingParameters)==248);
ABI(processing_context_1976,sizeof(NeckProcessingContext)==1976);
ABI(processing_output_1616,sizeof(NeckProcessingOutput)==1616);
ABI(pose_632,sizeof(NeckPoseBatch)==632);
ABI(relative_diagnostic_200,sizeof(NeckRelativeDiagnostic)==200);
ABI(alarm_event_48,sizeof(NeckAlarmEvent)==48);
ABI(stats_124,sizeof(NeckAppStats)==124);
ABI(clock_24,sizeof(NeckTickClock)==24);
ABI(sequence_u64,sizeof(((NeckFrame *)0)->sequence)==8);
ABI(frame_time_u64,sizeof(((NeckFrame *)0)->frame_time_us)==8);
ABI(sample_time_u64,sizeof(((NeckNodeSample *)0)->sample_time_us)==8);
ABI(decision_time_u64,sizeof(((NeckQualityOutput *)0)->decision_time_us)==8);
ABI(valid_until_u64,sizeof(((NeckPoseBatch *)0)->valid_until_us)==8);
ABI(epoch_u32,sizeof(((NeckPoseBatch *)0)->invalidation_epoch)==4);
int neck_target_abi_compile_proof(void) { return 0; }
