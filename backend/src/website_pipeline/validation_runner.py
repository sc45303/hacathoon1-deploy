"""
Main entry point for the retrieval pipeline validation tool.

This script orchestrates the entire validation process by coordinating
all components: configuration, query processing, validation, comparison, and reporting.
"""
import argparse
import json
import sys
import logging
from typing import List
from datetime import datetime

from .retrieval_validator import RetrievalValidator
from .result_comparator import validate_retrieved_chunks
from .report_generator import generate_validation_report, save_report
from .validation_data_models import ValidationQuery, ValidationResult, RetrievedChunk
from .validation_config import ValidationConfig


def load_test_queries(queries_file: str) -> List[ValidationQuery]:
    """
    Load test queries from a JSON file.
    
    Args:
        queries_file: Path to the queries JSON file
        
    Returns:
        List of ValidationQuery objects
    """
    with open(queries_file, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    queries = []
    for item in data['queries']:
        query = ValidationQuery(
            id=item['id'],
            text=item['text'],
            type=item['type'],
            expected_section=item['expected_section'],
            metadata=item.get('metadata', {})
        )
        queries.append(query)
    
    return queries


def run_validation(config: ValidationConfig) -> ValidationReport:
    """
    Run the complete validation process.

    Args:
        config: Validation configuration

    Returns:
        ValidationReport with results
    """
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    logger.info(f"Starting validation with config: collection={config.collection_name}, top_k={config.top_k}, threshold={config.threshold}")

    # Initialize validator
    validator = RetrievalValidator(
        collection_name=config.collection_name,
        top_k=config.top_k
    )

    logger.info(f"Loading test queries from {config.test_queries_file}")
    # Load test queries
    test_queries = load_test_queries(config.test_queries_file)
    logger.info(f"Loaded {len(test_queries)} test queries")

    # Filter queries by type if specified
    if config.query_types:
        original_count = len(test_queries)
        test_queries = [q for q in test_queries if q.type in config.query_types]
        logger.info(f"Filtered queries by types {config.query_types}: {original_count} -> {len(test_queries)} queries")

    results = []
    successful_count = 0

    # Process each query
    for i, query in enumerate(test_queries, 1):
        logger.info(f"Processing query {i}/{len(test_queries)}: {query.id} ({query.type})")
        print(f"Validating query {i}/{len(test_queries)}: {query.id} ({query.type}) - {query.text[:50]}...")

        try:
            # Get retrieved chunks from validator
            retrieved_data = validator.validate_single_query(query.text, config.top_k)
            logger.debug(f"Retrieved {len(retrieved_data)} chunks for query {query.id}")

            # Convert retrieved data to RetrievedChunk objects
            retrieved_chunks = []
            for item in retrieved_data:
                chunk = RetrievedChunk(
                    id=item['id'],
                    content=item['content'],
                    url=item['url'],
                    position=item.get('position', 0),
                    relevance_score=item['relevance_score'],
                    source_metadata=item.get('source_metadata', {})
                )
                retrieved_chunks.append(chunk)

            # Validate the retrieved chunks against expected content
            correctness, accuracy_score, failure_reason, comparison_details = validate_retrieved_chunks(
                retrieved_chunks,
                query.expected_section
            )

            if correctness:
                successful_count += 1
                logger.debug(f"Query {query.id} PASSED with accuracy {accuracy_score:.3f}")
            else:
                logger.debug(f"Query {query.id} FAILED with accuracy {accuracy_score:.3f}, reason: {failure_reason}")

            # Create validation result
            result = ValidationResult(
                query_id=query.id,
                retrieved_chunks=retrieved_chunks,
                accuracy_score=accuracy_score,
                correctness=correctness,
                comparison_details=comparison_details,
                failure_reason=failure_reason
            )

            results.append(result)
            print(f"  Result: {'PASS' if correctness else 'FAIL'} (Score: {accuracy_score:.3f})")

        except Exception as e:
            logger.error(f"Error processing query {query.id}: {e}")
            # Create a failed result for this query
            result = ValidationResult(
                query_id=query.id,
                retrieved_chunks=[],
                accuracy_score=0.0,
                correctness=False,
                comparison_details={},
                failure_reason=str(e)
            )
            results.append(result)
            print(f"  Result: FAIL (Error: {e})")

    logger.info(f"Validation completed. {successful_count}/{len(test_queries)} queries passed.")

    # Calculate query type breakdown
    type_counts = {}
    type_correct = {}

    for query, result in zip(test_queries, results):
        q_type = query.type
        if q_type not in type_counts:
            type_counts[q_type] = 0
            type_correct[q_type] = 0

        type_counts[q_type] += 1
        if result.correctness:
            type_correct[q_type] += 1

    query_type_breakdown = {}
    for q_type in type_counts:
        count = type_counts[q_type]
        correct = type_correct[q_type]
        accuracy = (correct / count) * 100 if count > 0 else 0.0

        query_type_breakdown[q_type] = {
            "accuracy": accuracy,
            "count": count,
            "correct": correct
        }

    logger.info(f"Query type breakdown: {query_type_breakdown}")

    # Generate final report
    report = generate_validation_report(
        validation_results=results,
        total_queries=len(test_queries),
        successful_queries=successful_count,
        query_type_breakdown=query_type_breakdown
    )

    logger.info(f"Final accuracy: {report.accuracy_percentage:.2f}%")
    return report


def main():
    """
    Main function that parses arguments and runs the validation.
    """
    parser = argparse.ArgumentParser(description="Validate the retrieval pipeline")
    parser.add_argument("--config-path", type=str, default=".env",
                        help="Path to configuration file (default: '.env')")
    parser.add_argument("--collection-name", type=str, default="reg-embedding",
                        help="Qdrant collection name to validate (default: 'reg-embedding')")
    parser.add_argument("--test-queries-file", type=str, default="validation/test_queries.json",
                        help="Path to file containing test queries (default: 'validation/test_queries.json')")
    parser.add_argument("--output-path", type=str, default="validation_report.json",
                        help="Path to save validation report (default: 'validation_report.json')")
    parser.add_argument("--output-format", type=str, default="json", choices=["json", "text"],
                        help="Output format (json, text) (default: 'json')")
    parser.add_argument("--threshold", type=float, default=0.9,
                        help="Minimum accuracy threshold (default: 0.9)")
    parser.add_argument("--top-k", type=int, default=5,
                        help="Number of top results to retrieve (default: 5)")
    parser.add_argument("--query-types", type=str, default="factual,conceptual,section-specific",
                        help="Comma-separated list of query types to test (default: 'factual,conceptual,section-specific')")
    
    args = parser.parse_args()
    
    # Parse query types from comma-separated string
    query_types = [qt.strip() for qt in args.query_types.split(',')] if args.query_types else []
    
    # Create configuration
    config = ValidationConfig(
        collection_name=args.collection_name,
        top_k=args.top_k,
        threshold=args.threshold,
        query_types=query_types,
        test_queries_file=args.test_queries_file,
        output_path=args.output_path,
        output_format=args.output_format,
        config_path=args.config_path
    )
    
    # Validate configuration
    errors = config.validate()
    if errors:
        print("Configuration errors found:")
        for error in errors:
            print(f"  - {error}")
        sys.exit(3)  # Exit with code 3 for invalid input parameters
    
    try:
        # Run validation
        report = run_validation(config)
        
        # Save report
        save_report(report, config.output_path, config.output_format)
        
        print(f"\nValidation completed. Report saved to {config.output_path}")
        print(f"Overall accuracy: {report.accuracy_percentage:.2f}%")
        
        # Exit with appropriate code based on threshold
        if report.accuracy_percentage >= (config.threshold * 100):
            sys.exit(0)  # Success
        else:
            print(f"Accuracy ({report.accuracy_percentage:.2f}%) is below threshold ({config.threshold * 100}%)")
            sys.exit(1)  # Below threshold
        
    except Exception as e:
        print(f"Validation failed with error: {str(e)}")
        sys.exit(2)  # Runtime error


if __name__ == "__main__":
    main()