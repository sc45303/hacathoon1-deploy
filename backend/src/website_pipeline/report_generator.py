"""
Validation report generation.

This module handles generating comprehensive validation reports that summarize
accuracy metrics, failure cases, and query type breakdowns.
"""
import json
import logging
from datetime import datetime
from typing import List, Dict, Any
from .validation_data_models import ValidationReport, ValidationResult, RetrievedChunk
from .result_comparator import validate_retrieved_chunks


logger = logging.getLogger(__name__)


def generate_validation_report(
    validation_results: List[ValidationResult],
    total_queries: int,
    successful_queries: int,
    query_type_breakdown: Dict[str, Dict[str, Any]]
) -> ValidationReport:
    """
    Generate a comprehensive validation report.
    
    Args:
        validation_results: List of individual validation results
        total_queries: Total number of queries tested
        successful_queries: Number of successful queries
        query_type_breakdown: Accuracy breakdown by query type
        
    Returns:
        ValidationReport object with all required information
    """
    accuracy_percentage = (successful_queries / total_queries) * 100 if total_queries > 0 else 0.0
    
    # Generate failure analysis
    failure_analysis = analyze_failures(validation_results)
    
    report = ValidationReport(
        id=f"validation_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
        date=datetime.now(),
        total_queries=total_queries,
        successful_queries=successful_queries,
        accuracy_percentage=accuracy_percentage,
        detailed_results=validation_results,
        failure_analysis=failure_analysis,
        query_type_breakdown=query_type_breakdown
    )
    
    return report


def analyze_failures(validation_results: List[ValidationResult]) -> Dict[str, Any]:
    """
    Analyze validation failures to identify common patterns.
    
    Args:
        validation_results: List of validation results to analyze
        
    Returns:
        Dictionary with failure analysis
    """
    failure_counts = {}
    failure_reasons = []
    total_failures = 0
    
    for result in validation_results:
        if not result.correctness:
            total_failures += 1
            if result.failure_reason:
                failure_reasons.append(result.failure_reason)
                # Count common failure reasons
                if result.failure_reason in failure_counts:
                    failure_counts[result.failure_reason] += 1
                else:
                    failure_counts[result.failure_reason] = 1
    
    # Get top failure reasons
    sorted_failures = sorted(failure_counts.items(), key=lambda x: x[1], reverse=True)
    top_failures = dict(sorted_failures[:5])  # Top 5 failure reasons
    
    return {
        "total_failures": total_failures,
        "failure_percentage": (total_failures / len(validation_results)) * 100 if validation_results else 0.0,
        "top_failure_reasons": top_failures,
        "all_failure_reasons": failure_reasons
    }


def calculate_query_type_breakdown(validation_results: List[ValidationResult]) -> Dict[str, Dict[str, Any]]:
    """
    Calculate accuracy breakdown by query type.
    
    Args:
        validation_results: List of validation results
        
    Returns:
        Dictionary with accuracy breakdown by query type
    """
    type_counts = {}
    type_correct = {}
    
    # Assuming query_type can be inferred from the original ValidationQuery ID or other context
    # In practice, you might need to pass the original query info to get the type
    for result in validation_results:
        # This is a simplified approach - a real implementation would map result.query_id to query type
        query_type = "unknown"  # This would be determined based on the source of the query
        
        if query_type not in type_counts:
            type_counts[query_type] = 0
            type_correct[query_type] = 0
        
        type_counts[query_type] += 1
        if result.correctness:
            type_correct[query_type] += 1
    
    breakdown = {}
    for query_type in type_counts:
        count = type_counts[query_type]
        correct = type_correct[query_type]
        accuracy = (correct / count) * 100 if count > 0 else 0.0
        
        breakdown[query_type] = {
            "accuracy": accuracy,
            "count": count,
            "correct": correct
        }
    
    return breakdown


def save_report(report: ValidationReport, output_path: str, output_format: str = "json"):
    """
    Save the validation report to a file.
    
    Args:
        report: The validation report to save
        output_path: Path to save the report
        output_format: Format to save in ('json' or 'text')
    """
    if output_format == "json":
        report_dict = {
            "id": report.id,
            "date": report.date.isoformat(),
            "total_queries": report.total_queries,
            "successful_queries": report.successful_queries,
            "accuracy_percentage": report.accuracy_percentage,
            "detailed_results": [
                {
                    "query_id": result.query_id,
                    "retrieved_chunks": [
                        {
                            "id": chunk.id,
                            "content": chunk.content,
                            "url": chunk.url,
                            "position": chunk.position,
                            "relevance_score": chunk.relevance_score,
                            "source_metadata": chunk.source_metadata
                        }
                        for chunk in result.retrieved_chunks
                    ],
                    "accuracy_score": result.accuracy_score,
                    "correctness": result.correctness,
                    "comparison_details": result.comparison_details,
                    "failure_reason": result.failure_reason
                }
                for result in report.detailed_results
            ],
            "failure_analysis": report.failure_analysis,
            "query_type_breakdown": report.query_type_breakdown
        }
        
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(report_dict, f, indent=2, ensure_ascii=False)
    elif output_format == "text":
        # Create a human-readable text report
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(f"Validation Report ID: {report.id}\n")
            f.write(f"Date: {report.date}\n")
            f.write(f"Total Queries: {report.total_queries}\n")
            f.write(f"Successful Queries: {report.successful_queries}\n")
            f.write(f"Accuracy Percentage: {report.accuracy_percentage:.2f}%\n\n")
            
            f.write("Query Type Breakdown:\n")
            for query_type, stats in report.query_type_breakdown.items():
                f.write(f"  {query_type}: {stats['accuracy']:.2f}% accuracy ({stats['correct']}/{stats['count']})\n")
            
            f.write("\nFailure Analysis:\n")
            f.write(f"  Total Failures: {report.failure_analysis['total_failures']}\n")
            f.write(f"  Failure Percentage: {report.failure_analysis['failure_percentage']:.2f}%\n")
            f.write("  Top Failure Reasons:\n")
            for reason, count in report.failure_analysis['top_failure_reasons'].items():
                f.write(f"    {reason}: {count} occurrences\n")
            
            f.write("\nDetailed Results:\n")
            for i, result in enumerate(report.detailed_results, 1):
                f.write(f"\n{i}. Query ID: {result.query_id}\n")
                f.write(f"   Correctness: {'PASS' if result.correctness else 'FAIL'}\n")
                f.write(f"   Accuracy Score: {result.accuracy_score:.3f}\n")
                if result.failure_reason:
                    f.write(f"   Failure Reason: {result.failure_reason}\n")
                f.write(f"   Retrieved Chunks: {len(result.retrieved_chunks)}\n")
                for j, chunk in enumerate(result.retrieved_chunks, 1):
                    f.write(f"     {j}. Score: {chunk.relevance_score:.3f}, URL: {chunk.url}\n")
                    f.write(f"        Content Preview: {chunk.content[:100]}...\n")
    else:
        raise ValueError(f"Unsupported output format: {output_format}")